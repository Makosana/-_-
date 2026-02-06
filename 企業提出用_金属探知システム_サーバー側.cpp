// test_heightnum.cpp  (GUI + TCPサーバ クライアント画像/電圧受信版)

#ifdef _WIN32                          // Windows環境の場合のみ有効にするプリプロセッサ分岐
// OpenCV が内部で windows.h を include する前に winsock2 を入れておく
#define _WINSOCKAPI_                   // windows.h 内で winsock.h がincludeされるのを防ぐためのマクロ
#include <winsock2.h>                  // WinSock2 の基本ヘッダ
#include <ws2tcpip.h>                  // TCP/IP 関連の追加ヘッダ
#pragma comment(lib, "Ws2_32.lib")     // Ws2_32.lib をリンクする指定
using socklen_t = int;                 // Unix互換の socklen_t 型を int として定義
using socket_t = SOCKET;               // ソケット型を統一的に socket_t として扱う
#define CLOSESOCK(s) closesocket(s)    // ソケットクローズ用マクロ（Windows版）
#define LAST_SOCK_ERR() WSAGetLastError() // 最後のソケットエラーコード取得マクロ（Windows版）
#define SOCK_INVALID INVALID_SOCKET    // 無効ソケットの定義（Windows版）
#define SOCK_ERROR   SOCKET_ERROR      // ソケットエラーを表す定数（Windows版）
#else                                  // ここからは Linux / Unix 系 OS 用
#include <sys/socket.h>                // ソケット関連ヘッダ
#include <sys/types.h>                 // 型定義ヘッダ
#include <netinet/in.h>                // インターネットアドレス構造体など
#include <netinet/tcp.h>               // TCP_NODELAY 等 TCP オプション
#include <arpa/inet.h>                 // inet_addr 等の関数
#include <unistd.h>                    // close などのUNIXシステムコール
#include <errno.h>                     // errno（エラーコード）取得用
using socket_t = int;                  // ソケットを int として扱う
#define CLOSESOCK(s) close(s)          // ソケットクローズ用マクロ（POSIX版）
#define LAST_SOCK_ERR() errno          // 最後のエラー番号取得マクロ（POSIX版）
#define SOCK_INVALID (-1)              // 無効ソケットの定義（POSIX版）
#define SOCK_ERROR   (-1)              // ソケットエラーを表す値（POSIX版）
#endif                                 // OS分岐ここまで

#include <opencv2/opencv.hpp>          // OpenCV のメインヘッダ
#define CVUI_IMPLEMENTATION            // cvui の実装をこのファイルで行う指定
#include "cvui.h"                      // GUI ライブラリ cvui のヘッダ

#include <cstdio>                      // printf などC標準入出力
#include <algorithm>                   // std::max, std::min など
#include <string>                      // std::string
#include <vector>                      // std::vector
#include <utility>                     // std::pair 等
#include <cmath>                       // 四捨五入などの数学関数
#include <cstdint>                     // uint8_t, uint32_t など固定幅整数
#include <iostream>                    // C++ 標準入出力ストリーム
#include <chrono>                      // FPS計測用
#include <thread>                      // 【追加】スレッド（並列処理用）
#include <mutex>                       // 【追加】排他制御
#include <atomic>                      // 【追加】スレッド安全な共有変数
#include <queue>                       // 【追加】キュー（データ受け渡し用）

using namespace cv;                    // OpenCV 名前空間を省略して使用
using std::string;                     // std::string を string として使用
using std::vector;                     // std::vector を vector として使用

static const char* WIN_MAIN = "cvui camera + heatmap"; // メインウィンドウのタイトル文字列

//=== レイアウト定数 ===
const Size   CANVAS_SIZE(1400, 800);                 // メインキャンバスのサイズ（幅1400, 高さ800）
const Scalar WHITE(255, 255, 255);                   // 白色（BGR = 255,255,255）
const Rect   VIEW_ROI(50, 50, 800, 600);             // 画像表示領域（左上(50,50), 幅800, 高さ600）
const Size   BTN_SIZE(200, 80);                      // ボタン共通サイズ（幅200, 高さ80）
const int    BTN_X = CANVAS_SIZE.width - 50 - BTN_SIZE.width; // 右端から50px空けたボタンX座標
const int    BTN_Y0 = 50;                            // how to use ボタンのY座標
const int    BTN_GAP = 10;                           // ボタン同士の縦間隔
const int    BTN_Y1 = BTN_Y0 + BTN_SIZE.height + BTN_GAP; // input height ボタンのY座標
const int    BTN_Y2 = BTN_Y1 + BTN_SIZE.height + BTN_GAP; // setup ボタンY
const int    BTN_Y3 = BTN_Y2 + BTN_SIZE.height + BTN_GAP; // start ボタンY
const int    BTN_Y4 = BTN_Y3 + BTN_SIZE.height + BTN_GAP; // stop ボタンY
const int    BTN_Y5 = BTN_Y4 + BTN_SIZE.height + BTN_GAP; // reset ボタンY
const int    BTN_Y6 = BTN_Y5 + BTN_SIZE.height + BTN_GAP; // shutdown ボタンY
const int    BTN_Y7 = BTN_Y6 + BTN_SIZE.height + BTN_GAP; // change background ボタンY

//=== グリッド / ヒートマップ ===
const int GRID_W = 40;     // ヒートマップ横方向セル数（800/10）
const int GRID_H = 30;     // ヒートマップ縦方向セル数（600/10）
const int CELL   = 20;     // 1セルのピクセルサイズ（20x20）
int heat[GRID_W][GRID_H] = { 0 }; // 各セルのレベル（0..5）を格納する配列を0初期化

//=== 円検出パラメータ ===
struct CircleDetectParam {
    int outer_r_min = 20;        // 外円探索の最小半径
    int outer_r_max = 60;        // 外円探索の最大半径
    int inner_r_min = 8;         // 内円探索の最小半径
    int inner_r_max = 25;        // 内円探索の最大半径
    double dp       = 1.2;       // HoughCircles の分解能係数
    double minDist  = 40;        // 検出される円同士の最小距離
    double canny    = 120;       // Canny の上限閾値（エッジ検出用）
    double acc      = 35;        // Hough 累積値の閾値
} cd;                             // 上記構造体のインスタンス cd を生成

//=== 高さ→半径テーブル（指定値に忠実） ===============================
// アンカー： {高さcm, 半径}
// 40→(outer=60, inner=40)
// 50→(50,40)
// 60→(40,30)
// 70→(30,25)
// 80→(30,20)
// 90→(25,20)
// 100→(25,20)
struct RadiusAnchor { int h; int r; }; // 高さと半径のペアを表す構造体

// piecewise 線形補間（端はクランプ）
static int interpByAnchors(int h, const std::vector<RadiusAnchor>& a) {
    if (a.empty()) return 0;                          // アンカーが空なら0を返す
    if (h <= a.front().h) return a.front().r;        // 最小高さ以下は先頭の半径を返す
    if (h >= a.back().h)  return a.back().r;         // 最大高さ以上は末尾の半径を返す
    for (size_t i = 1; i < a.size(); ++i) {          // アンカー区間を順に確認
        if (h <= a[i].h) {                           // h が現在のアンカー範囲内に収まる場合
            const auto& L = a[i - 1];                // 左側アンカー
            const auto& R = a[i];                    // 右側アンカー
            const double t = double(h - L.h) / double(R.h - L.h); // 線形補間用の割合t
            return (int)std::lround(L.r + (R.r - L.r) * t);       // 線形補間して四捨五入
        }
    }
    return a.back().r;                               // ループを抜けた場合は最大値を返す
}

// 高さ(cm)から HoughCircles の探索半径範囲を設定
// テーブルの目標半径±バンド幅で絞り込み（検出が不安定なら帯域を広げてください）
void applyHeightToCircleParams(int height_cm) {
    // 指定テーブル
    static const std::vector<RadiusAnchor> outerA = {
        {40,60}, {50,50}, {60,40}, {70,30}, {80,30}, {90,25}, {100,25}
    };                                              // 外円用の高さ→半径アンカー
    static const std::vector<RadiusAnchor> innerA = {
        {40,40}, {50,40}, {60,30}, {70,25}, {80,20}, {90,20}, {100,20}
    };                                              // 内円用の高さ→半径アンカー

    int h = std::max(40, std::min(100, height_cm)); // 高さを40〜100の範囲にクランプ

    const int targetOuter = interpByAnchors(h, outerA); // 外円目標半径を取得
    const int targetInner = interpByAnchors(h, innerA); // 内円目標半径を取得

    // 探索帯域（必要に応じて ±2 → ±3 などに調整可）
    const int BW_OUT = 2;                           // 外円半径の許容バンド幅
    const int BW_IN  = 2;                           // 内円半径の許容バンド幅

    cd.outer_r_min = std::max(5, targetOuter - BW_OUT);            // 外円最小半径を設定（最低5）
    cd.outer_r_max = std::max(cd.outer_r_min + 1, targetOuter + BW_OUT); // 外円最大半径を設定

    cd.inner_r_min = std::max(3, targetInner - BW_IN);             // 内円最小半径を設定（最低3）
    cd.inner_r_max = std::max(cd.inner_r_min + 1, targetInner + BW_IN);   // 内円最大半径を設定
}

//=== ユーティリティ ===
void putSafeText(Mat& img, const string& text, Point org,
    double scale = 0.6, Scalar color = Scalar(60, 60, 60),
    int thickness = 2, int maxLen = 200) {
    string s = text;                                 // 表示用文字列 s に text をコピー
    if ((int)s.size() > maxLen) s = s.substr(0, maxLen - 3) + "..."; // 最大長を超えた場合は末尾を省略
    putText(img, s, org, FONT_HERSHEY_SIMPLEX, scale, color, thickness); // 画像上に文字列を描画
}

int voltageToLevel(int voltage_value) {
    printf("[voltageToLevel] voltage_value=%d\n", voltage_value);
    if (voltage_value < 820) return 1;             // 電圧が2800未満ならレベル1
    else if (voltage_value < 840) return 2;        // 2800〜2500未満ならレベル2
    else if (voltage_value < 860) return 3;        // 2500〜3500未満ならレベル3
    else if (voltage_value < 870) return 4;        // 3500〜4500未満ならレベル4
    else return 5;                                  // 4500以上ならレベル5
}

bool toGrid(const Point& p, int& gx, int& gy) {
    if (p.x < 0 || p.y < 0 || p.x >= 800 || p.y >= 600) return false; // 800x600 の範囲外ならfalse
    gx = p.x / CELL; // 0..79                                  // x座標をセル番号に変換
    gy = p.y / CELL; // 0..59                                  // y座標をセル番号に変換
    return (gx >= 0 && gx < GRID_W && gy >= 0 && gy < GRID_H); // グリッド範囲内なら true
}

Mat buildHeatOverlay() {
    Mat overlay(600, 800, CV_8UC3, Scalar(0, 0, 0));           // 黒色で600x800の画像（オーバーレイ）を作成
    for (int y = 0; y < GRID_H; ++y) {                        // 縦方向（各セル）ループ
        for (int x = 0; x < GRID_W; ++x) {                    // 横方向（各セル）ループ
            const int v = heat[x][y];                         // 該当セルのレベル値を取得
            if (v == 0) continue;                             // 0なら描画しない（透明扱い）
            Scalar color;                                     // 色を格納する変数
            switch (v) {                                      // レベルに応じて色を決定
            case 1: color = Scalar(0,   0, 255); break;   // 赤（BGR）
            case 2: color = Scalar(0, 165, 255); break;   // オレンジ
            case 3: color = Scalar(0, 255, 255); break;   // 黄
            case 4: color = Scalar(0, 255,   0); break;   // 緑
            case 5: color = Scalar(255, 0,   0); break;   // 青
            default: color = Scalar(0, 0, 0); break;      // 想定外は黒
            }
            rectangle(overlay, Rect(x * CELL, y * CELL, CELL, CELL), // セル領域に
                      color, FILLED);                                // 塗りつぶし矩形を描画
        }
    }
    return overlay;                                            // 作成したオーバーレイ画像を返す
}

bool detectOuterCircle(const Mat& bgr, Vec3f& out) {
    Mat gray, blur;                                           // グレースケール・ぼかし用Mat
    cvtColor(bgr, gray, COLOR_BGR2GRAY);                      // BGR画像をグレースケールに変換
    GaussianBlur(gray, blur, Size(9, 9), 2);                  // ガウシアンぼかしをかける
    vector<Vec3f> circles;                                    // 検出された円を格納する配列
    HoughCircles(blur, circles, HOUGH_GRADIENT, cd.dp, cd.minDist,
                 cd.canny, cd.acc, cd.outer_r_min, cd.outer_r_max); // 外円検出
    if (circles.empty()) return false;                        // 1つも円がない場合は false
    size_t idx = 0;                                           // 一番大きい円のインデックス
    for (size_t i = 1; i < circles.size(); ++i)               // 全ての円を走査
        if (circles[i][2] > circles[idx][2]) idx = i;         // 半径が最大の円を選ぶ
    out = circles[idx];                                       // 結果を out に格納
    return true;                                              // 検出成功
}

bool detectInnerCircleInOuter(const Mat& bgr, const Vec3f& outer, Vec3f& inner) {
    int cx = cvRound(outer[0]), cy = cvRound(outer[1]), rr = cvRound(outer[2]); // 外円の中心座標と半径を整数化
    int x0 = std::max(0,   cx - rr + 5);                    // 外円内のROI左端（少し内側にオフセット）
    int y0 = std::max(0,   cy - rr + 5);                    // 外円内のROI上端（少し内側）
    int x1 = std::min(799, cx + rr - 5);                    // 外円内のROI右端（画面外に出ないように）
    int y1 = std::min(599, cy + rr - 5);                    // 外円内のROI下端（画面外に出ないように）
    if (x1 <= x0 || y1 <= y0) return false;                 // ROIが成立しない場合は false

    Rect roi(x0, y0, x1 - x0 + 1, y1 - y0 + 1);             // 内円探索用の矩形領域を定義
    Mat crop = bgr(roi).clone();                            // BGR画像からROIを切り出してコピー

    Mat gray, blur;                                         // ROIのグレースケール・ぼかし用
    cvtColor(crop, gray, COLOR_BGR2GRAY);                   // ROIをグレースケールに変換
    GaussianBlur(gray, blur, Size(9, 9), 2);                // ガウシアンぼかしをかける

    vector<Vec3f> circles;                                  // 検出された内円を格納する配列
    HoughCircles(blur, circles, HOUGH_GRADIENT, cd.dp, cd.minDist * 0.5,
                 cd.canny, cd.acc, cd.inner_r_min, cd.inner_r_max); // 内円を検出（距離条件少し緩め）
    if (circles.empty()) return false;                      // 円が見つからなければ false
    size_t idx = 0;                                         // 最大半径の円インデックス
    for (size_t i = 1; i < circles.size(); ++i)             // 全ての候補から
        if (circles[i][2] > circles[idx][2]) idx = i;       // 半径が最大のものを選ぶ

    inner[0] = circles[idx][0] + x0;                        // ROI座標から画像全体座標へ変換（x）
    inner[1] = circles[idx][1] + y0;                        // 同上（y）
    inner[2] = circles[idx][2];                             // 内円の半径
    return true;                                            // 検出成功
}

bool buttonDisabledAware(Mat& canvas, int x, int y, const string& label, bool enabled) {
    if (enabled) return cvui::button(canvas, x, y, BTN_SIZE.width, BTN_SIZE.height, label);
    // enabled が true なら通常の cvui::button を描いて押下結果を返す
    Rect rc(x, y, BTN_SIZE.width, BTN_SIZE.height);         // ボタン領域の矩形
    rectangle(canvas, rc, Scalar(220, 220, 220), FILLED);   // グレーでボタン背景を描画（無効表示）
    rectangle(canvas, rc, Scalar(180, 180, 180), 1);        // 薄い枠線を描画
    int base = 0;                                           // テキスト描画用ベースライン
    Size sz = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.7, 2, &base); // ラベル文字列のサイズを取得
    putText(canvas, label,
        Point(x + (BTN_SIZE.width - sz.width) / 2,
              y + (BTN_SIZE.height + sz.height) / 2 - 6),
        FONT_HERSHEY_SIMPLEX, 0.7, Scalar(150, 150, 150), 2); // グレー文字でラベルを描画
    return false;                                           // 無効ボタンなので常に false
}

//=== How To Use ===
void runHowToUseWindow() {
    const int CANVAS_W = 1400, CANVAS_H = 800;              // HowToUseウィンドウのサイズ
    const int IMG_W = 800, IMG_H = 600;                     // 説明画像の表示サイズ
    const int OFF_X = (CANVAS_W - IMG_W) / 2;               // 画像表示左上X（中央寄せ）
    const int OFF_Y = (CANVAS_H - IMG_H) / 2 - 30;          // 画像表示左上Y（やや上寄せ）
    const int BTN_W = 100, BTN_H = 100;                     // Back/Nextボタンサイズ
    const int BTN_Y = OFF_Y + (IMG_H / 2);                  // Back/NextボタンのY座標
    const int BTN_X_BACK = OFF_X - 150;                     // BackボタンのX座標（左側）
    const int BTN_X_NEXT = OFF_X + 850;                     // NextボタンのX座標（右側）
    const int CLOSE_W = 100, CLOSE_H = 100;                 // Closeボタンサイズ
    const int CLOSE_X = CANVAS_W - CLOSE_W - 50;            // CloseボタンX（右下から50px）
    const int CLOSE_Y = CANVAS_H - CLOSE_H - 50;            // CloseボタンY（下から50px）

    const string win = "How to use";                        // ウィンドウタイトル
    namedWindow(win);                                       // ウィンドウ生成
    cvui::init(win);                                        // cvui の初期化

    vector<string> files;                                   // 説明画像ファイル名のリスト
    for (int i = 1; i <= 20; ++i) {                         // explan01〜explan20 を順にチェック
        char name[64];                                      // ファイル名バッファ
        std::snprintf(name, sizeof(name), "explan%02d.png", i); // explanXX.png という名前を作る
        Mat probe = imread(name);                           // 試しに画像を読み込む
        if (probe.empty()) break;                           // 読み込めなければ存在しないと判断して終了
        files.emplace_back(name);                           // 存在したファイル名をリストに追加
    }
    if (files.empty()) files = { "explan01.png","explan02.png","explan03.png" };
    // 一枚も見つからなかった場合のデフォルト候補

    vector<Mat> show(files.size());                         // 表示用にリサイズした画像配列
    for (size_t i = 0; i < files.size(); ++i) {             // 各ファイルについて
        Mat img = imread(files[i], IMREAD_COLOR);           // カラーで読み込み
        if (img.empty()) {                                  // 読み込み失敗時
            Mat ph(IMG_H, IMG_W, CV_8UC3, Scalar(220, 220, 220)); // 灰色のダミー画像
            putSafeText(ph, "NOT FOUND: " + files[i],
                        Point(30, IMG_H / 2), 1.0, Scalar(0, 0, 255), 2); // ファイル未発見メッセージ描画
            show[i] = ph;                                   // ダミーを保存
        } else {
            resize(img, show[i], Size(IMG_W, IMG_H), 0, 0, INTER_AREA); // 800x600にリサイズして保存
        }
    }

    int cur = 0;                                            // 現在表示中の画像インデックス
    while (true) {                                          // HowToUseウィンドウのメインループ
        Mat canvas(CANVAS_H, CANVAS_W, CV_8UC3, WHITE);     // 背景を白で初期化
        show[cur].copyTo(canvas(Rect(OFF_X, OFF_Y, IMG_W, IMG_H))); // 現在画像をキャンバス中央に貼り付け
        putSafeText(canvas, "Current: " + files[cur], Point(OFF_X, OFF_Y - 10));
        // 現在表示中のファイル名を表示

        if (cur > 0) {                                      // 先頭でなければBackボタンを有効化
            if (cvui::button(canvas, BTN_X_BACK, BTN_Y, BTN_W, BTN_H, "Back")) --cur;
            // Backを押すと前の画像へ
        } else {                                            // 先頭のときは押せないグレー表示
            rectangle(canvas, Rect(BTN_X_BACK, BTN_Y, BTN_W, BTN_H), Scalar(200, 200, 200), FILLED);
            putSafeText(canvas, "Back", Point(BTN_X_BACK + 22, BTN_Y + 35),
                        0.7, Scalar(120, 120, 120), 2);
        }
        if (cur < (int)show.size() - 1) {                   // 最後でなければNextボタン有効
            if (cvui::button(canvas, BTN_X_NEXT, BTN_Y, BTN_W, BTN_H, "Next")) ++cur;
            // Nextを押すと次の画像へ
        } else {                                            // 最後のときはグレー表示
            rectangle(canvas, Rect(BTN_X_NEXT, BTN_Y, BTN_W, BTN_H), Scalar(200, 200, 200), FILLED);
            putSafeText(canvas, "Next", Point(BTN_X_NEXT + 20, BTN_Y + 35),
                        0.7, Scalar(120, 120, 120), 2);
        }

        if (cvui::button(canvas, CLOSE_X, CLOSE_Y, CLOSE_W, CLOSE_H, "Close")) break;
        // Closeボタンでループを抜ける

        cvui::update(win);                                  // cvui 状態更新
        cvui::imshow(win, canvas);                          // ウィンドウに描画
        int key = waitKey(20);                              // キー入力待ち（20ms）
        if (key == 27 || key == 'q' || key == 'Q') break;   // ESC / q / Q で閉じる
    }
    destroyWindow(win);                                     // ウィンドウを破棄
}

//=== Change Background ===
void runChangeBackground(Mat& background) {
    const int CANVAS_W = 1400, CANVAS_H = 800;              // ウィンドウサイズ
    const int IMG_W = 600, IMG_H = 450;                     // プレビュー画像のサイズ
    const int imgX = (CANVAS_W - IMG_W) / 2;                // 画像の左上X（中央寄せ）
    const int imgY = (CANVAS_H - IMG_H) / 2 - 20;           // 画像の左上Y（やや上寄せ）
    const int BTN_LR_W = 100, BTN_LR_H = 100;               // back/nextボタンサイズ
    const int MARGIN = 20;                                  // 余白
    const int leftX = imgX - MARGIN - BTN_LR_W;             // back ボタンX
    const int rightX = imgX + IMG_W + MARGIN;               // next ボタンX
    const int lrY = imgY + (IMG_H - BTN_LR_H) / 2;          // back/nextボタンY（画像中央高さ）
    const int BTN_BIG_W = 200, BTN_BIG_H = 80;              // enter/closeボタンサイズ
    const int enterX = (CANVAS_W - BTN_BIG_W) / 2;          // enterボタンX（中央）
    const int enterY = CANVAS_H - 50 - BTN_BIG_H;           // enterボタンY（下から50）
    const int closeX = CANVAS_W - 50 - BTN_BIG_W;           // closeボタンX（右下から50）
    const int closeY = enterY;                              // closeボタンY（enterと同じ高さ）

    const string win = "Change background";                 // ウィンドウタイトル
    namedWindow(win);                                       // ウィンドウ生成
    cvui::init(win);                                        // cvui初期化

    vector<string> files;                                   // 背景候補ファイル名リスト
    for (int i = 1; i <= 100; ++i) {                        // image01〜image100 を探索
        char nameJ[64], nameP[64], namePNG[64];             // 拡張子違いの候補
        std::snprintf(nameJ, sizeof(nameJ), "image%02d.jpg", i);
        std::snprintf(nameP, sizeof(nameP), "image%02d.jpeg", i);
        std::snprintf(namePNG, sizeof(namePNG), "image%02d.png", i);
        Mat j = imread(nameJ), p = imread(nameP), g = imread(namePNG); // 3種類試し読み
        if (j.empty() && p.empty() && g.empty()) {          // 全て空ならファイル無し
            if (i == 1) continue;                           // 最初から無い場合は次へ（念のため）
            else break;                                     // 途中から無くなったらループ終了
        }
        if (!j.empty())      files.emplace_back(nameJ);     // .jpg があれば採用
        else if (!p.empty()) files.emplace_back(nameP);     // .jpeg があれば採用
        else                 files.emplace_back(namePNG);   // それ以外は .png を採用
    }
    if (files.empty()) files = { "image01.jpg","image02.jpg","image03.jpg" };
    // 一枚も見つからなければデフォルト候補

    vector<Mat> originals(files.size());                    // オリジナル画像
    vector<Mat> previews(files.size());                     // プレビューサイズに縮小
    for (size_t i = 0; i < files.size(); ++i) {             // 各ファイルごとに
        originals[i] = imread(files[i], IMREAD_COLOR);      // オリジナルを読み込み
        if (originals[i].empty()) {                         // 読み込めなければダミー画像
            Mat ph(IMG_H, IMG_W, CV_8UC3, Scalar(200, 200, 200));
            putSafeText(ph, "NOT FOUND: " + files[i],
                        Point(30, IMG_H / 2), 1.0, Scalar(0, 0, 255), 2);
            previews[i] = ph;                               // ダミーをプレビューにセット
        } else {
            resize(originals[i], previews[i], Size(IMG_W, IMG_H), 0, 0, INTER_AREA);
            // プレビューサイズに縮小
        }
    }

    int index = 0;                                          // 現在の候補インデックス
    while (true) {                                          // ウィンドウのメインループ
        Mat canvas(CANVAS_H, CANVAS_W, CV_8UC3, WHITE);     // キャンバスを白で初期化
        previews[index].copyTo(canvas(Rect(imgX, imgY, IMG_W, IMG_H)));
        // 現在のプレビュー画像を中央に貼り付け

        if (cvui::button(canvas, leftX, lrY, BTN_LR_W, BTN_LR_H, "back"))
            index = (index == 0 ? (int)previews.size() - 1 : index - 1);
        // backボタン：先頭なら末尾へ、それ以外は1つ前へ

        if (cvui::button(canvas, rightX, lrY, BTN_LR_W, BTN_LR_H, "next"))
            index = (index + 1) % (int)previews.size();
        // nextボタン：次のインデックスへ（末尾なら先頭にループ）

        if (cvui::button(canvas, enterX, enterY, BTN_BIG_W, BTN_BIG_H, "enter")) {
            // enterボタン：この画像を背景として確定
            if (!originals[index].empty()) background = originals[index].clone();
            else {
                background = Mat(CANVAS_H, CANVAS_W, CV_8UC3, Scalar(230, 230, 230));
                putSafeText(background, "Selected image not found.", Point(40, 60));
            }
            break;                                          // ループを抜ける
        }
        if (cvui::button(canvas, closeX, closeY, BTN_BIG_W, BTN_BIG_H, "close")) break;
        // closeボタン：何も変更せず終了

        cvui::update(win);                                  // cvui 状態更新
        cvui::imshow(win, canvas);                          // 画面描画
        int key = waitKey(20);                              // キー入力待ち（20ms）
        if (key == 27) break;                               // ESC で閉じる
    }
    destroyWindow(win);                                     // ウィンドウを破棄
}

//=== Shutdown ===
bool runShutdownConfirm() {
    const int CANVAS_W = 1400, CANVAS_H = 800;              // シャットダウン確認ウィンドウサイズ
    const string win = "Shutdown";                          // ウィンドウタイトル
    namedWindow(win);                                       // ウィンドウ作成
    cvui::init(win);                                        // cvui初期化

    const int BTN_W = 200, BTN_H = 80;                      // Yes/Noボタンサイズ
    const int BTN_Y = CANVAS_H - 50 - BTN_H;                // ボタンY（下から50）
    const int GAP = 60;                                     // ボタン間の隙間
    const int BTN_X_YES = CANVAS_W / 2 - BTN_W - GAP / 2;   // YesボタンX（中央より左）
    const int BTN_X_NO  = CANVAS_W / 2 + GAP / 2;           // NoボタンX（中央より右）
    const string msg = "Do you want to shut down?";         // 確認メッセージ

    while (true) {                                          // 確認ダイアログループ
        Mat canvas(CANVAS_H, CANVAS_W, CV_8UC3, WHITE);     // 白背景

        int base = 0;                                       // テキストベースライン
        double scale = 1.8;                                 // テキストサイズ
        int thick = 3;                                      // テキスト太さ
        Size sz = getTextSize(msg, FONT_HERSHEY_SIMPLEX, scale, thick, &base);
        // メッセージ文字サイズを取得
        Point org((CANVAS_W - sz.width) / 2, CANVAS_H / 2); // 中央に配置する原点座標
        putText(canvas, msg, org, FONT_HERSHEY_SIMPLEX, scale, Scalar(0, 0, 0), thick);
        // メッセージを描画

        if (cvui::button(canvas, BTN_X_YES, BTN_Y, BTN_W, BTN_H, "yes")) {
            // yes ボタン：true を返して終了
            destroyWindow(win);
            return true;
        }
        if (cvui::button(canvas, BTN_X_NO, BTN_Y, BTN_W, BTN_H, "no")) {
            // no ボタン：false を返して終了
            destroyWindow(win);
            return false;
        }

        cvui::update(win);                                  // cvui 更新
        cvui::imshow(win, canvas);                          // 描画
        int key = waitKey(20);                              // キー入力待ち
        if (key == 27) {                                    // ESC でキャンセル扱い
            destroyWindow(win);
            return false;
        }
    }
}

//=== Input Height（上/下の三角ボタン + 直接入力 + enter/close） ===
// 戻り値: first=確定したか, second=高さ(40..100)
std::pair<bool, int> runInputHeightWindow(int initialValue = 50) {
    const int CANVAS_W = 1400, CANVAS_H = 800;              // ウィンドウサイズ
    const string win = "Input height";                      // ウィンドウタイトル
    namedWindow(win);                                       // ウィンドウ作成
    cvui::init(win);                                        // cvui 初期化

    Mat frame(CANVAS_H, CANVAS_W, CV_8UC3);                 // 描画用フレーム画像
    auto clampRange = [](int v) { return std::max(40, std::min(100, v)); };
    // 高さを40〜100に制限するラムダ

    int    heightVal = clampRange(initialValue);            // 初期高さ値をクランプして設定
    string input     = std::to_string(heightVal);           // 表示用文字列
    bool   isEditing = false;                               // 直接入力中かどうかのフラグ

    const Rect box(600, 300, 200, 100);                     // 数値表示/入力ボックス領域

    auto triangleButton = [&](const vector<Point>& pts, double& lastClickTime,
                              double debounceMs = 130) {
        // 三角形ボタンを描画し、押されたかどうかを返すラムダ
        Point mouse(cvui::mouse().x, cvui::mouse().y);      // マウス位置取得
        bool inside = pointPolygonTest(pts, mouse, false) >= 0; // マウスが三角形内か判定
        Scalar color = inside ? Scalar(60, 60, 60) : Scalar(0, 0, 0); // ホバーで色変更
        fillConvexPoly(frame, pts, color, LINE_AA);         // 三角形を描画

        double now = (double)getTickCount() / getTickFrequency() * 1000.0;
        // 現在時刻（ms）取得
        if (inside && cvui::mouse(cvui::IS_DOWN) &&
            (now - lastClickTime > debounceMs)) {
            // 三角形内でマウス押下 & 一定時間経過なら押下とみなす
            lastClickTime = now;                            // 最終クリック時刻を更新
            return true;                                    // ボタン押下を示す
        }
        return false;                                       // 押されていない
    };

    double lastClickUp = 0.0, lastClickDown = 0.0;          // 上/下ボタンの最終クリック時刻

    const int BTN_W = 200, BTN_H = 80;                      // enter/closeボタンサイズ
    const int enterX = 600,  enterY = 670;                  // enterボタン位置
    const int closeX = 1150, closeY = 670;                  // closeボタン位置

    while (true) {                                          // 入力ウィンドウのメインループ
        frame.setTo(WHITE);                                 // 背景を白にクリア
        cvui::update();                                     // cvui更新

        cvui::text(frame, 640, 120, "Input   height", 0.8, 0x000000);
        // タイトルテキスト表示

        Scalar boxColor = isEditing ? Scalar(100, 180, 255) : Scalar(0, 0, 0);
        // 編集中はボックス枠を青っぽく、それ以外は黒
        rectangle(frame, box, boxColor, 2);                 // 入力ボックス枠線を描画

        vector<Point> upTri = {                             // 上向き三角ボタンの3頂点
            {box.x + box.width / 2 - 20, box.y - 10},
            {box.x + box.width / 2 + 20, box.y - 10},
            {box.x + box.width / 2,      box.y - 40}
        };
        vector<Point> downTri = {                           // 下向き三角ボタンの3頂点
            {box.x + box.width / 2 - 20, box.y + box.height + 10},
            {box.x + box.width / 2 + 20, box.y + box.height + 10},
            {box.x + box.width / 2,      box.y + box.height + 40}
        };

        if (!isEditing && triangleButton(upTri, lastClickUp)) {
            // 編集中でなく、上三角ボタンが押された場合
            heightVal = clampRange(heightVal + 10);         // 高さを+10してクランプ
            input = std::to_string(heightVal);              // 表示文字列更新
        }
        if (!isEditing && triangleButton(downTri, lastClickDown)) {
            // 編集中でなく、下三角ボタンが押された場合
            heightVal = clampRange(heightVal - 10);         // 高さを-10してクランプ
            input = std::to_string(heightVal);              // 表示文字列更新
        }

        if (!isEditing &&
            cvui::iarea(box.x, box.y, box.width, box.height) &&
            cvui::mouse(cvui::CLICK)) {
            // ボックスがクリックされたら編集モードに入る
            isEditing = true;                               // 直接入力モードON
            input.clear();                                  // 文字列初期化
        }

        int key = waitKey(1);                               // キー入力チェック（1ms）
        if (isEditing) {                                    // 編集モードの場合
            if (key >= '0' && key <= '9') {
                // 0〜9キーなら入力に追加（3桁まで）
                if (input.size() < 3) input.push_back((char)key);
            }
            else if ((key == 8 || key == 127) && !input.empty()) {
                // バックスペース/Deleteで1文字削除
                input.pop_back();
            }
            else if (key == 13 || key == 27) {
                // Enter or ESC で編集終了
                isEditing = false;
                if (!input.empty()) {
                    try {
                        heightVal = clampRange(std::stoi(input)); // 入力文字列を数値に変換
                    } catch (...) {}                              // 失敗時は無視
                }
                input = std::to_string(heightVal);             // 最終値を文字列に反映
            }
        }

        string displayText = input.empty() ? " " : input;     // 空文字の場合はスペースを表示
        int baseline = 0;                                     // テキストベースライン
        Size textSize = getTextSize(displayText, FONT_HERSHEY_SIMPLEX, 2, 4, &baseline);
        // 数値表示のサイズ取得
        Point textOrg(box.x + (box.width - textSize.width) / 2,
                      box.y + (box.height + textSize.height) / 2 - baseline);
        // ボックス中央に配置する原点計算
        putText(frame, displayText, textOrg,
                FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 0, 0), 4, LINE_AA);
        // 数値を描画

        cvui::text(frame, box.x + box.width + 40, box.y + 60, "cm", 0.8, 0x000000);
        // 単位 "cm" をボックス右に表示

        if (cvui::button(frame, enterX, enterY, BTN_W, BTN_H, "enter")) {
            // enter ボタンが押されたら値確定して true を返す
            destroyWindow(win);
            return { true, heightVal };
        }
        if (cvui::button(frame, closeX, closeY, BTN_W, BTN_H, "close")) {
            // close ボタンが押されたらキャンセル扱いで false を返す
            destroyWindow(win);
            return { false, heightVal };
        }

        imshow(win, frame);                                  // ウィンドウに描画
        if (key == 27 && !isEditing) {
            // 編集中でなく ESC が押されたらキャンセル
            destroyWindow(win);
            return { false, heightVal };
        }
    }
}

//======================================================================
// ここから TCP/IP サーバ部（test_server.txt をベースに GUI 用に組み込み）
//======================================================================

// --- Protocol constants ---
static constexpr uint8_t CMD_CONNECT   = 0x01;              // 接続開始コマンド
static constexpr uint8_t CMD_GET_IMAGE = 0x02;              // 画像取得コマンド
static constexpr uint8_t CMD_GET_VALUE = 0x03;              // 電圧値取得コマンド
static constexpr uint8_t CMD_STOP      = 0x04;              // 測定停止コマンド
static constexpr uint8_t CMD_SHUTDOWN  = 0x05;              // クライアント側シャットダウンコマンド

static constexpr uint8_t ACK           = 0x06;              // ACK (アクノリッジ) コード

static constexpr uint8_t TYPE_IMAGE    = 0x11;              // 画像パケットタイプ
static constexpr uint8_t TYPE_VALUE    = 0x12;              // 値パケットタイプ

static constexpr int     SERVER_PORT   = 10000;             // サーバで待ち受けるTCPポート番号

//======================================================================
// 【追加】非同期受信用のグローバル変数
//======================================================================
struct ReceivedFrame {
    cv::Mat image;          // 受信した画像
    uint32_t voltage;       // 電圧値（画像と同時に取得）
    bool valid;             // 有効かどうか
};

static std::mutex g_frameMutex;                    // フレームバッファの排他制御
static ReceivedFrame g_latestFrame = { cv::Mat(), 0, false };  // 最新フレーム
static std::atomic<bool> g_receiverRunning{ false };  // 受信スレッド動作フラグ
static std::atomic<bool> g_sessionActive{ false };    // セッションがアクティブか
static std::atomic<bool> g_connectionError{ false };  // 接続エラーフラグ
static std::atomic<int> g_receivedFrameCount{ 0 };    // 受信フレーム数（FPS計測用）

// ------------ helper ------------
bool sendAll(socket_t s, const void* data, size_t len) {
    const uint8_t* p = static_cast<const uint8_t*>(data);   // 送信データをバイトポインタに変換
    size_t sent = 0;                                        // 既に送信したバイト数
    while (sent < len) {                                    // 全て送信するまでループ
        int n = ::send(s, reinterpret_cast<const char*>(p + sent),
                       static_cast<int>(len - sent), 0);    // 残りを送信
        if (n == SOCK_ERROR) return false;                  // エラーなら false
        if (n == 0) return false;                           // 0 なら切断とみなす
        sent += static_cast<size_t>(n);                     // 送信済みバイト数を更新
    }
    return true;                                            // 全バイト送信成功
}

bool recvAll(socket_t s, void* data, size_t len) {
    uint8_t* p = static_cast<uint8_t*>(data);               // 受信バッファをバイトポインタに
    size_t recvd = 0;                                       // 受信済みバイト数
    while (recvd < len) {                                   // 指定サイズ受信するまでループ
        int n = ::recv(s, reinterpret_cast<char*>(p + recvd),
                       static_cast<int>(len - recvd), 0);   // 残りを受信
        if (n == SOCK_ERROR) return false;                  // エラーなら false
        if (n == 0) return false; // connection closed       // 0なら切断
        recvd += static_cast<size_t>(n);                    // 受信済みバイト数更新
    }
    return true;                                            // 全バイト受信成功
}

bool sendByte(socket_t s, uint8_t b) {
    return sendAll(s, &b, 1);                               // 単一バイトを送信するユーティリティ
}

bool recvByte(socket_t s, uint8_t& b) {
    return recvAll(s, &b, 1);                               // 単一バイトを受信するユーティリティ
}

bool expectAck(socket_t s) {
    uint8_t b = 0;                                          // 受信用1バイト変数
    if (!recvByte(s, b)) return false;                      // 受信失敗なら false
    return b == ACK;                                        // 受け取った値が ACK なら true
}

bool recvUint32N(socket_t s, uint32_t& out_host) {
    uint32_t netv = 0;                                      // ネットワークバイトオーダの値
    if (!recvAll(s, &netv, sizeof(netv))) return false;     // 4バイト受信
    out_host = ntohl(netv);                                 // ネット→ホストバイトオーダに変換
    return true;                                            // 成功
}

// クライアント接続待ち（1回だけ）
bool waitClientConnection(socket_t& out_client) {
#ifdef _WIN32
    WSADATA wsa;                                            // WinSock 初期化用構造体
    if (WSAStartup(MAKEWORD(2,2), &wsa) != 0) {             // WinSock をバージョン2.2で初期化
        std::cerr << "WSAStartup failed.\n";                // 失敗時メッセージ
        return false;                                       // falseを返して終了
    }
#endif
    std::cout << "[*] ポート " << SERVER_PORT << " でクライアントからの接続要求を待機します...\n";
    // 接続待ちを標準出力に表示

    socket_t listen_fd = ::socket(AF_INET, SOCK_STREAM, 0); // TCPソケット作成
    if (listen_fd == SOCK_INVALID) {                        // 失敗チェック
        std::cerr << "socket() failed: " << LAST_SOCK_ERR() << "\n";
        return false;
    }

    int yes = 1;                                            // SO_REUSEADDR用の値
#ifdef _WIN32
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR,
               (const char*)&yes, sizeof(yes));             // Windows版 setsockopt
#else
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR,
               &yes, sizeof(yes));                          // POSIX版 setsockopt
#endif

    sockaddr_in addr{};                                     // サーバアドレス構造体
    addr.sin_family = AF_INET;                              // IPv4 を指定
    addr.sin_addr.s_addr = htonl(INADDR_ANY);               // 任意のインターフェースで受付
    addr.sin_port = htons(SERVER_PORT);                     // ポート番号をネットワークバイトオーダに変換
    if (::bind(listen_fd, reinterpret_cast<sockaddr*>(&addr),
               sizeof(addr)) == SOCK_ERROR) {               // ソケットにアドレスを割り当て
        std::cerr << "bind() failed: " << LAST_SOCK_ERR() << "\n";
        CLOSESOCK(listen_fd);                               // バインド失敗時ソケットをクローズ
#ifdef _WIN32
        WSACleanup();                                       // WinSock後始末
#endif
        return false;
    }

    if (::listen(listen_fd, 1) == SOCK_ERROR) {             // 接続待ち状態にする（最大1接続）
        std::cerr << "listen() failed: " << LAST_SOCK_ERR() << "\n";
        CLOSESOCK(listen_fd);                               // ソケットクローズ
#ifdef _WIN32
        WSACleanup();                                       // WinSock後始末
#endif
        return false;
    }

    sockaddr_in cli{};                                      // クライアントアドレス構造体
    socklen_t clen = sizeof(cli);                           // アドレス構造体サイズ
    socket_t cli_fd = ::accept(listen_fd,
                               reinterpret_cast<sockaddr*>(&cli),
                               &clen);                      // クライアント接続を受け付け
    if (cli_fd == SOCK_INVALID) {                           // 失敗チェック
        std::cerr << "accept() failed: " << LAST_SOCK_ERR() << "\n";
        CLOSESOCK(listen_fd);                               // 待受ソケットをクローズ
#ifdef _WIN32
        WSACleanup();                                       // WinSock後始末
#endif
        return false;
    }
    CLOSESOCK(listen_fd);  
    
    // TCP_NODELAYを設定（遅延削減 / Nagle無効化）
    {
        int flag = 1;
#ifdef _WIN32
        if (setsockopt(cli_fd, IPPROTO_TCP, TCP_NODELAY, (const char*)&flag, sizeof(flag)) == SOCK_ERROR) {
            std::cerr << "setsockopt(TCP_NODELAY) failed: " << LAST_SOCK_ERR() << "\n";
        }
#else
        if (setsockopt(cli_fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag)) == SOCK_ERROR) {
            std::cerr << "setsockopt(TCP_NODELAY) failed: " << LAST_SOCK_ERR() << "\n";
        }
#endif
    }                                 // 以降はcli_fdだけ使うので待受ソケットは閉じる

    std::cout << "[*] 接続: " << inet_ntoa(cli.sin_addr)
              << ":"    << ntohs(cli.sin_port) << "\n";
    // 接続してきたクライアントのIPとポートを表示

    out_client = cli_fd;                                    // 呼び出し元へクライアントソケットを渡す
    return true;                                            // 成功
}

// セッション開始 (0x01 -> ACK)
bool startSession(socket_t cli_fd) {
    if (!sendByte(cli_fd, CMD_CONNECT)) return false;        // 0x01 CONNECT コマンド送信
    if (!expectAck(cli_fd)) return false;                    // ACKを期待して受信
    return true;                                             // 成功
}

// セッション終了 (0x04 -> ACK)
bool stopSession(socket_t cli_fd) {
    if (!sendByte(cli_fd, CMD_STOP)) return false;           // 0x04 STOP コマンド送信
    if (!expectAck(cli_fd)) return false;                    // ACKを受信
    return true;                                             // 成功
}

// シャットダウン (0x05 -> ACK)
bool shutdownRemote(socket_t cli_fd) {
    if (!sendByte(cli_fd, CMD_SHUTDOWN)) return false;       // 0x05 SHUTDOWN コマンド送信
    if (!expectAck(cli_fd)) return false;                    // ACKを受信
    return true;                                             // 成功
}

// 画像取得 (0x02 プロトコル)
//  - 必要なら startSession で 0x01 を送ってから使用する
bool fetchRemoteImage(socket_t cli_fd, Mat& dst) {
    if (!sendByte(cli_fd, CMD_GET_IMAGE)) return false;      // 画像要求コマンド(0x02)送信

    // 0x06
    if (!expectAck(cli_fd)) return false;                    // 最初のACKを待つ

    // 0x11
    uint8_t typ = 0;                                         // 受信タイプコード
    if (!recvByte(cli_fd, typ)) return false;                // 1バイト受信
    if (typ != TYPE_IMAGE) {                                 // 期待するタイプと違う場合
        std::cerr << "TYPE mismatch: expected 0x11, got 0x"
                  << std::hex << (int)typ << std::dec << "\n";
        return false;
    }

    // 画像サイズ
    uint32_t img_size = 0;                                   // 画像データサイズ
    if (!recvUint32N(cli_fd, img_size)) return false;        // 4バイトのサイズを受信
    if (img_size == 0 || img_size > (64u * 1024u * 1024u)) { // サイズの妥当性チェック（0や64MB超を禁止）
        std::cerr << "invalid image size: " << img_size << "\n";
        return false;
    }

    std::vector<uchar> buf(img_size);                        // 画像データ格納用バッファ
    if (!recvAll(cli_fd, buf.data(), buf.size())) return false; // 指定サイズ分受信

    Mat img = imdecode(buf, IMREAD_COLOR);                   // メモリ上のJPEGデータをデコード
    if (img.empty()) {                                       // 失敗チェック
        std::cerr << "imdecode failed.\n";
        return false;
    }
    if (img.size() != Size(800, 600)) {                      // サイズが800x600でない場合
        resize(img, img, Size(800, 600));                    // 800x600にリサイズ
    }
    dst = img;                                               // 呼び出し元へ画像を渡す

    // 最後に ACK 送信
    if (!sendByte(cli_fd, ACK)) return false;                // 受信完了のACK(0x06)を返す
    return true;                                             // 正常終了
}

// 値取得 (0x03 プロトコル)
bool fetchRemoteValue(socket_t cli_fd, uint32_t& outVal) {
    if (!sendByte(cli_fd, CMD_GET_VALUE)) return false;      // 値取得コマンド(0x03)送信

    if (!expectAck(cli_fd)) return false;                    // ACKを受信

    uint8_t typ = 0;                                         // パケットタイプ受信用
    if (!recvByte(cli_fd, typ)) return false;                // 1バイト受信
    if (typ != TYPE_VALUE) {                                 // 値タイプ(0x12)でなければエラー
        std::cerr << "TYPE mismatch: expected 0x12, got 0x"
                  << std::hex << (int)typ << std::dec << "\n";
        return false;
    }

    uint32_t v = 0;                                          // 受信用32bit整数
    if (!recvUint32N(cli_fd, v)) return false;               // 値を受信
    outVal = v;                                              // 結果を呼び出し元へ

    if (!sendByte(cli_fd, ACK)) return false;                // 最後にACKを返す
    return true;                                             // 成功
}

// 画像だけ or 画像+値を一度に取得
//  - sessionActive == false のときは 0x01 でセッション開始
bool remoteCapture(socket_t cli_fd, bool& sessionActive,
                   Mat& outImg, uint32_t* outValOpt) {
    if (!sessionActive) {                                    // セッション未開始なら
        if (!startSession(cli_fd)) {                         // 0x01 を送って開始
            std::cerr << "[proto] startSession failed.\n";
            return false;
        }
        sessionActive = true;                                // セッションフラグをON
    }
    if (!fetchRemoteImage(cli_fd, outImg)) {                 // 画像取得
        sessionActive = false;                               // 失敗時はセッションを無効化
        return false;
    }
    if (outValOpt) {                                         // 電圧値も必要な場合
        uint32_t v = 0;                                      // 一時変数
        if (!fetchRemoteValue(cli_fd, v)) {                  // 値取得
            sessionActive = false;                           // 失敗時はセッションを無効化
            return false;
        }
        *outValOpt = v;                                      // 取得した値を格納
    }
    return true;                                             // 正常に画像（＋値）取得完了
}

//======================================================================
// 【追加】非同期受信スレッド
// サーバーの処理とは独立して、連続的に画像を受信し続ける
//======================================================================
void receiverThread(socket_t cli_fd) {
    std::cout << "[receiver] Thread started.\n";
    
    // ソケットに受信タイムアウトを設定（stopで固まらないように）
#ifdef _WIN32
    DWORD timeout_ms = 500;  // 500msタイムアウト
    setsockopt(cli_fd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout_ms, sizeof(timeout_ms));
#else
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 500000;  // 500ms
    setsockopt(cli_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
#endif
    
    // まずセッションを開始（まだ開始されていない場合のみ）
    if (!g_sessionActive) {
        if (!sendByte(cli_fd, CMD_CONNECT)) {
            std::cerr << "[receiver] startSession send failed.\n";
            g_connectionError = true;
            return;
        }
        if (!expectAck(cli_fd)) {
            std::cerr << "[receiver] startSession ACK failed.\n";
            g_connectionError = true;
            return;
        }
        g_sessionActive = true;
    }
    
    std::cout << "[receiver] Session started, beginning capture loop.\n";
    
    while (g_receiverRunning) {
        // 画像要求を送信
        if (!sendByte(cli_fd, CMD_GET_IMAGE)) {
            if (!g_receiverRunning) break;  // 停止要求なら正常終了
            std::cerr << "[receiver] send CMD_GET_IMAGE failed.\n";
            g_connectionError = true;
            break;
        }
        
        // ACKを待つ
        if (!expectAck(cli_fd)) {
            if (!g_receiverRunning) break;  // 停止要求なら正常終了
            std::cerr << "[receiver] expectAck failed.\n";
            g_connectionError = true;
            break;
        }
        
        // タイプを受信
        uint8_t typ = 0;
        if (!recvByte(cli_fd, typ)) {
            if (!g_receiverRunning) break;
            std::cerr << "[receiver] recv TYPE failed.\n";
            g_connectionError = true;
            break;
        }
        if (typ != TYPE_IMAGE) {
            std::cerr << "[receiver] TYPE mismatch: got 0x" << std::hex << (int)typ << std::dec << "\n";
            g_connectionError = true;
            break;
        }
        
        // 画像サイズを受信
        uint32_t img_size = 0;
        if (!recvUint32N(cli_fd, img_size)) {
            if (!g_receiverRunning) break;
            std::cerr << "[receiver] recv size failed.\n";
            g_connectionError = true;
            break;
        }
        if (img_size == 0 || img_size > (64u * 1024u * 1024u)) {
            std::cerr << "[receiver] invalid image size: " << img_size << "\n";
            g_connectionError = true;
            break;
        }
        
        // 画像データを受信
        std::vector<uchar> buf(img_size);
        if (!recvAll(cli_fd, buf.data(), buf.size())) {
            if (!g_receiverRunning) break;
            std::cerr << "[receiver] recvAll failed.\n";
            g_connectionError = true;
            break;
        }
        
        // ACKを返す（プロトコル維持のため）
        if (!sendByte(cli_fd, ACK)) {
            if (!g_receiverRunning) break;
            std::cerr << "[receiver] send ACK failed.\n";
            g_connectionError = true;
            break;
        }
        
        // JPEGデコード
        Mat img = imdecode(buf, IMREAD_COLOR);
        if (img.empty()) {
            std::cerr << "[receiver] imdecode failed.\n";
            continue;  // デコード失敗は無視して次へ
        }
        
        // サイズ調整
        if (img.size() != Size(800, 600)) {
            resize(img, img, Size(800, 600));
        }
        
        // === ここで電圧値も取得 ===
        uint32_t voltage = 0;
        if (!sendByte(cli_fd, CMD_GET_VALUE)) {
            if (!g_receiverRunning) break;
            g_connectionError = true;
            break;
        }
        if (!expectAck(cli_fd)) {
            if (!g_receiverRunning) break;
            g_connectionError = true;
            break;
        }
        uint8_t vtyp = 0;
        if (!recvByte(cli_fd, vtyp)) {
            if (!g_receiverRunning) break;
            g_connectionError = true;
            break;
        }
        if (vtyp != TYPE_VALUE) {
            g_connectionError = true;
            break;
        }
        if (!recvUint32N(cli_fd, voltage)) {
            if (!g_receiverRunning) break;
            g_connectionError = true;
            break;
        }
        if (!sendByte(cli_fd, ACK)) {
            if (!g_receiverRunning) break;
            g_connectionError = true;
            break;
        }
        
        // 最新フレームを更新（ロックして安全に書き込み）
        {
            std::lock_guard<std::mutex> lock(g_frameMutex);
            g_latestFrame.image = img.clone();
            g_latestFrame.voltage = voltage;
            g_latestFrame.valid = true;
        }
        g_receivedFrameCount++;
        
        // 最初のフレーム受信時にログ出力
        if (g_receivedFrameCount == 1) {
            std::cout << "[receiver] First frame received successfully!\n";
        }
    }
    
    // タイムアウトを解除（元に戻す）
#ifdef _WIN32
    DWORD timeout_off = 0;
    setsockopt(cli_fd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout_off, sizeof(timeout_off));
#else
    struct timeval tv_off;
    tv_off.tv_sec = 0;
    tv_off.tv_usec = 0;
    setsockopt(cli_fd, SOL_SOCKET, SO_RCVTIMEO, &tv_off, sizeof(tv_off));
#endif
    
    std::cout << "[receiver] Thread stopped. Total frames: " << g_receivedFrameCount << "\n";
}

// 【追加】最新フレームを取得（メインスレッドから呼ぶ、待ち時間ほぼゼロ）
bool getLatestFrame(Mat& outImg, uint32_t& outVoltage) {
    std::lock_guard<std::mutex> lock(g_frameMutex);
    if (!g_latestFrame.valid) return false;
    outImg = g_latestFrame.image.clone();
    outVoltage = g_latestFrame.voltage;
    return true;
}

//======================================================================
// メイン
//======================================================================
int main() {
    // --- ネットワーク: クライアント接続待ち ---
    socket_t cli_fd = SOCK_INVALID;                          // クライアントソケット初期値（無効）
    if (!waitClientConnection(cli_fd)) {                     // クライアント接続を待つ
        std::cerr << "client connection failed.\n";          // 失敗時メッセージ
        return 1;                                            // 異常終了
    }
    bool sessionActive = false;                              // 0x01 セッション開始済みかどうかを表すフラグ

    Mat canvas(CANVAS_SIZE, CV_8UC3, WHITE);                 // メインキャンバス画像（白で初期化）
    Mat background;                                          // 背景画像（change backgroundで差し替え）

    Mat origin;   // 800x600で保持（setup/reset時に取得）
    Mat image;    // 800x600で処理用（start中に毎フレーム取得）
    Mat display;  // 合成結果（ヒートマップなどを重ねた表示用画像）

    // FPS計測（受信・処理できたフレームをカウント）
    double fps = 0.0;
    auto lastFpsTime = std::chrono::steady_clock::now();
    int lastReceivedCount = 0;                              // 前回のフレーム受信数

    bool running   = false;                                  // 測定ループが動作中かどうか
    bool hasOrigin = false;                                  // origin画像が取得済みかどうか
    bool canStart  = false;                                  // startボタンを押せるか
    bool canStop   = false;                                  // stopボタンを押せるか
    bool canReset  = false;                                  // resetボタンを押せるか

    // ★ 高さは「未設定」から開始。未設定の間は setup を無効化する
    bool heightSet = false;                                  // 高さが設定済みかどうか
    int  height_cm = 0; // 表示用、未設定時は "--" 相当で扱う

    // 【追加】非同期受信スレッド用
    std::thread receiverThreadHandle;

    namedWindow(WIN_MAIN);                                   // メインウィンドウ作成
    cvui::init(WIN_MAIN);                                    // cvui 初期化

    while (true) {                                           // メインループ
        // 背景
        if (!background.empty()) background.copyTo(canvas);  // 背景画像があればそれをキャンバスにコピー
        else canvas.setTo(WHITE);                            // なければ白で塗りつぶし

        // 【修正】接続エラーチェック
        if (g_connectionError) {
            running = false;
            canStop = false;
            canReset = true;
            g_receiverRunning = false;
            if (receiverThreadHandle.joinable()) {
                receiverThreadHandle.join();
            }
            g_connectionError = false;
            putSafeText(canvas, "[Error] connection lost.", Point(60, 720), 0.7, Scalar(0, 0, 255), 2);
        }

        // 【修正】実行中: 非同期受信スレッドから最新フレームを取得して処理
        if (running) {
            Mat tmp;                                         // 受信用画像
            uint32_t value = 0;                              // 受信した電圧値
            
            // 【変更】非同期で受信済みの最新フレームを取得（待ち時間ほぼゼロ）
            if (getLatestFrame(tmp, value)) {
                image = tmp;                                 // image に最新の画像を保存

                // 【修正】FPS更新（受信スレッドのカウントを使用）
                auto now = std::chrono::steady_clock::now();
                auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastFpsTime).count();
                if (elapsedMs >= 1000) {
                    int currentCount = g_receivedFrameCount.load();
                    int delta = currentCount - lastReceivedCount;
                    fps = delta * 1000.0 / (double)elapsedMs;
                    lastReceivedCount = currentCount;
                    lastFpsTime = now;
                }

                Vec3f outer, inner;                          // 外円・内円の情報
                bool gotOuter = detectOuterCircle(image, outer);          // 外円検出
                bool gotInner = gotOuter ? detectInnerCircleInOuter(image, outer, inner) : false;
                // 外円が見つかった場合のみ、その内部で内円を検出する

                if (gotOuter && gotInner) {                  // 両方の円が見つかった場合
                    Point marker(cvRound(inner[0]), cvRound(inner[1])); // 内円中心座標
                    int gx, gy;                              // グリッド座標
                    if (toGrid(marker, gx, gy)) {            // マーカーがグリッド内なら
                        int lvl = voltageToLevel((int)value);// 電圧値からレベルを取得
                        heat[gx][gy] = lvl;                  // 対応セルのヒートマップ値を更新
                        circle(image, Point(cvRound(outer[0]), cvRound(outer[1])),
                               cvRound(outer[2]), Scalar(0, 255, 0), 2);
                        // 外円を緑色で描画
                        circle(image, marker, cvRound(inner[2]),
                               Scalar(255, 0, 0), 2);
                        // 内円を青っぽい色で描画
                        circle(image, marker, 3, Scalar(0, 0, 255), FILLED);
                        // 中心点を赤い点で描画
                    }
                }
            }
            // 【削除】else節のエラー処理は g_connectionError で処理
        }

        // 合成表示
        if (!image.empty()) {                                // 最新画像がある場合
            Mat overlay = buildHeatOverlay();                // ヒートマップオーバーレイを作成
            addWeighted(image, 1.0, overlay, 0.35, 0.0, display);
            // imageとoverlayを合成（overlayを35%重ねる）
        } else if (!origin.empty()) {                        // imageがなくoriginだけある場合
            display = origin.clone();                        // originを表示用にコピー
        } else {
            display.release();                               // 何もなければ display を空にする
        }

        if (!display.empty()) {                              // display画像がある場合
            display.copyTo(canvas(VIEW_ROI));                // VIEW_ROI内に表示
        } else if (!origin.empty()) {                        // displayは空だがoriginはある場合
            origin.copyTo(canvas(VIEW_ROI));                 // originを表示
        } else {
            rectangle(canvas, VIEW_ROI, Scalar(200, 200, 200), 2);
            // 枠線だけ描いて
            putSafeText(canvas, "No image. Press 'setup' to capture from client.",
                        Point(VIEW_ROI.x + 20, VIEW_ROI.y + VIEW_ROI.height / 2));
            // メッセージを表示
        }

        // 仕切り
        line(canvas,
             Point(VIEW_ROI.x + VIEW_ROI.width + 30, 30),
             Point(VIEW_ROI.x + VIEW_ROI.width + 30,
                   CANVAS_SIZE.height - 30),
             Scalar(220, 220, 220), 2);
        // 画像領域とボタン領域を区切る縦線

        // 高さ表示
        if (heightSet) {                                    // 高さが設定されている場合
            char hbuf[64];                                  // 表示用バッファ
            std::snprintf(hbuf, sizeof(hbuf), "height: %d cm", height_cm);
            putSafeText(canvas, hbuf,
                        Point(BTN_X - 220, 30),
                        0.7, Scalar(40, 40, 40), 2);
            // 設定された高さを表示
        } else {
            putSafeText(canvas,
                        "height: -- (press 'input height' to set)",
                        Point(BTN_X - 300, 30),
                        0.7, Scalar(0, 0, 255), 2);
            // 未設定の場合の注意メッセージ
        }

        // FPS表示
        {
            char fpsBuf[64];
            std::snprintf(fpsBuf, sizeof(fpsBuf), "FPS: %.1f", fps);
            putSafeText(canvas, fpsBuf,
                        Point(VIEW_ROI.x, VIEW_ROI.y + VIEW_ROI.height + 25),
                        0.7, Scalar(0, 120, 0), 2);
        }

        // クライアント接続状態表示
        string status = (cli_fd != SOCK_INVALID)
                        ? "[Client] Connected"
                        : "[Client] Not connected";
        // 接続有無に応じてステータスメッセージを用意
        if (sessionActive) status += " (session active)";
        else               status += " (session idle)";
        // セッションが開始されているかどうかを付加
        putSafeText(canvas, status,
                    Point(50, CANVAS_SIZE.height - 20),
                    0.6,
                    (cli_fd != SOCK_INVALID)
                        ? Scalar(40, 120, 40)
                        : Scalar(0, 0, 255),
                    2);
        // クライアント状態を画面左下付近に表示（接続時は緑、未接続時は赤）

        // ボタン群
        if (buttonDisabledAware(canvas, BTN_X, BTN_Y0, "how to use", true)) {
            // how to use ボタン（常に有効）
            runHowToUseWindow();                             // 使い方ウィンドウを表示
            cvui::init(WIN_MAIN);                            // 戻ってきたらメインウィンドウを再初期化
        }

        // input height（確定時に heightSet = true & 半径反映）
        if (buttonDisabledAware(canvas, BTN_X, BTN_Y1, "input height", true)) {
            // input height ボタン（常に有効）
            auto res = runInputHeightWindow(heightSet ? height_cm : 50);
            // すでに設定済みならその値を初期値に、未設定なら50を初期値にしてダイアログ表示
            if (res.first) {                                 // enterで確定された場合
                height_cm = res.second;                      // 高さ値を更新
                heightSet = true;                            // 設定済みフラグON
                applyHeightToCircleParams(height_cm);        // 高さ→円探索半径に反映
            }
            cvui::init(WIN_MAIN);                            // メインウィンドウcvuiを再初期化
        }

        // setup は「高さが確定するまで」押せない
        const bool allowSetup = heightSet && (cli_fd != SOCK_INVALID);
        // 高さが設定済み & クライアント接続中なら setup を有効にする
        if (buttonDisabledAware(canvas, BTN_X, BTN_Y2, "setup", allowSetup)) {
            Mat tmp;                                         // 受信用一時画像
            // origin 取得だけなので値は不要
            if (!remoteCapture(cli_fd, sessionActive, tmp, nullptr)) {
                // 画像取得に失敗した場合
                putSafeText(canvas,
                            "[Error] failed to capture image from client.",
                            Point(60, 720), 0.7, Scalar(0, 0, 255), 2);
            } else {
                if (tmp.size() != Size(800, 600))
                    resize(tmp, origin, Size(800, 600));     // サイズを800x600に揃える
                else
                    origin = tmp;                            // そのまま origin に保存
                hasOrigin = true;                            // origin取得済みフラグON
                image.release();                             // imageはリセット
                display = origin.clone();                    // 表示用にoriginをコピー
                running  = false;                            // 測定を停止状態に
                canStart = true;                             // startボタンを有効にする
                canStop  = false;                            // stopボタンは無効
                canReset = false;                            // resetボタンは無効

                // heat マップ初期化
                for (int x = 0; x < GRID_W; ++x)
                    for (int y = 0; y < GRID_H; ++y)
                        heat[x][y] = 0;                      // すべてのセルを0クリア
                
                // 【追加】setup後はセッションを終了しておく（startで新しく開始するため）
                if (sessionActive) {
                    stopSession(cli_fd);
                    sessionActive = false;
                    g_sessionActive = false;
                }
            }
        }

        // start は origin がある & まだ走っていない & 接続済み
        const bool allowStart = hasOrigin && canStart && !running && (cli_fd != SOCK_INVALID);
        if (buttonDisabledAware(canvas, BTN_X, BTN_Y3, "start", allowStart)) {
            // startボタンが押された場合（条件を満たしているときのみ）
            
            // 【追加】非同期受信スレッドを開始
            g_receiverRunning = true;
            g_connectionError = false;
            g_receivedFrameCount = 0;
            g_latestFrame.valid = false;
            receiverThreadHandle = std::thread(receiverThread, cli_fd);
            
            running  = true;                                 // 測定ループ開始
            canStart = false;                                // startボタンは無効化
            canStop  = true;                                 // stopボタンを有効化
            canReset = false;                                // resetボタン無効

            // FPS計測をリセット
            fps = 0.0;
            lastReceivedCount = 0;
            lastFpsTime = std::chrono::steady_clock::now();
        }

        // stop は実行中のみ
        if (buttonDisabledAware(canvas, BTN_X, BTN_Y4, "stop", canStop)) {
            // stopボタンが押された場合
            
            // 【追加】まず受信スレッドを停止
            g_receiverRunning = false;
            if (receiverThreadHandle.joinable()) {
                receiverThreadHandle.join();
            }
            
            // プロトコル仕様に従って 0x04 を送信
            sessionActive = g_sessionActive.load();
            if (sessionActive) {                             // セッションが有効な場合のみ
                if (!stopSession(cli_fd)) {                  // STOPコマンド送信
                    putSafeText(canvas,
                                "[Error] failed to send STOP to client.",
                                Point(60, 720), 0.7, Scalar(0, 0, 255), 2);
                }
                sessionActive = false;                       // セッションフラグOFF
                g_sessionActive = false;
            }
            running  = false;                                // 測定停止
            canStop  = false;                                // stop無効
            canStart = true;                                 // 再度startを押せる
            canReset = true;                                 // resetも押せる
        }

        // reset: origin を取り直し & heat 初期化
        if (buttonDisabledAware(canvas, BTN_X, BTN_Y5, "reset",
                                canReset && (cli_fd != SOCK_INVALID))) {
            // resetボタンが押された場合（条件を満たしているとき）
            Mat tmp;                                         // 一時画像
            if (!remoteCapture(cli_fd, sessionActive, tmp, nullptr)) {
                // 画像取得失敗
                putSafeText(canvas,
                            "[Error] failed to capture image from client.",
                            Point(60, 720), 0.7, Scalar(0, 0, 255), 2);
            } else {
                if (tmp.size() != Size(800, 600))
                    resize(tmp, origin, Size(800, 600));     // 800x600にリサイズ
                else
                    origin = tmp;                            // そのままoriginに格納
                hasOrigin = true;                            // origin取得済み
                image.release();                             // 処理画像をクリア
                display = origin.clone();                    // 表示用にコピー

                for (int x = 0; x < GRID_W; ++x)
                    for (int y = 0; y < GRID_H; ++y)
                        heat[x][y] = 0;                      // ヒートマップを初期化

                canStart = true;                             // start可能
                canStop  = false;                            // stop不可
                canReset = false;                            // reset不可
            }
        }

        // shutdown: ボタン押下で 0x05 をクライアントに送り、アプリ終了
        if (buttonDisabledAware(canvas, BTN_X, BTN_Y6, "shutdown", true)) {
            // shutdownボタン（常に有効）
            bool yes = runShutdownConfirm();                 // シャットダウン確認ダイアログ
            if (yes) {                                       // yes が選択された場合
                // 【追加】受信スレッドを停止
                g_receiverRunning = false;
                if (receiverThreadHandle.joinable()) {
                    receiverThreadHandle.join();
                }
                
                if (cli_fd != SOCK_INVALID) {                // クライアント接続が有効なら
                    shutdownRemote(cli_fd); // エラーは無視 // クライアントへシャットダウンコマンド送信
                    CLOSESOCK(cli_fd);                       // ソケットを閉じる
                    cli_fd = SOCK_INVALID;                   // 無効値にリセット
                }
#ifdef _WIN32
                WSACleanup();                                // Windowsの場合 WinSock後始末
#endif
                destroyAllWindows();                         // 全てのウィンドウを破棄
                return 0;                                    // 正常終了
            } else {
                cvui::init(WIN_MAIN);                        // キャンセル時はcvuiを再初期化して継続
            }
        }

        // change background: 通信とは無関係
        if (buttonDisabledAware(canvas, BTN_X, BTN_Y7, "change background", true)) {
            // 背景変更ボタン（常に有効）
            runChangeBackground(background);                 // 背景選択ウィンドウを開く
            cvui::init(WIN_MAIN);                            // 戻ったらcvuiを再初期化
        }

        cvui::update(WIN_MAIN);                              // cvui状態更新
        cvui::imshow(WIN_MAIN, canvas);                      // メインウィンドウに描画
        int key = waitKey(1);                                // キー入力（1ms待つ）
        if (key == 27) break; // ESC で終了                  // ESCキーでメインループ終了
    }

    // 【追加】プログラム終了前にスレッドを停止
    g_receiverRunning = false;
    if (receiverThreadHandle.joinable()) {
        receiverThreadHandle.join();
    }

    if (cli_fd != SOCK_INVALID) {                            // クライアントソケットが有効なら
        CLOSESOCK(cli_fd);                                   // ソケットを閉じる
    }
#ifdef _WIN32
    WSACleanup();                                            // Windowsの場合 WinSock後始末
#endif
    destroyAllWindows();                                     // 全てのOpenCVウィンドウを閉じる
    return 0;                                                // 正常終了
}
