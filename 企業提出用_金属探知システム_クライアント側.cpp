// ============================================================================
// client_main.cpp - カメラ映像＆センサー値送信クライアント
// ============================================================================
//
// 【このプログラムの概要】
//   ラズパイのカメラで撮影した映像と、ADコンバータ(MCP3002)で読み取った
//   電圧値をサーバーに送信するクライアントプログラムです。
//
// 【必要なもの】
//   - Raspberry Pi（カメラ付き）
//   - MCP3002（ADコンバータIC）をSPI接続
//   - OpenCV（画像処理ライブラリ）
//   - wiringPi（GPIO/SPI制御ライブラリ）
//   - cvui.h（UIライブラリ、今回は使わないけど取り込みだけ）
//
// 【動作の流れ】
//   1. サーバーに接続する（接続できるまで何度でもリトライ）
//   2. サーバーからのコマンドを待つ
//   3. コマンドに応じて、画像や電圧値を送信する
//
// 【サーバーから来るコマンド一覧】
//   ● セッション外（最初の状態）
//     0x01: セッション開始 → ACK(0x06)を返す
//     0x05: シャットダウン → ACK(0x06)を返して終了
//
//   ● セッション中（0x01を受けた後）
//     0x02: 画像ください → カメラ画像をJPEGで送る
//     0x03: 電圧値ください → ADコンバータの値を送る
//     0x04: セッション終了 → ACK(0x06)を返してセッション外に戻る
//     0x05: シャットダウン → プログラム終了
//
// 【ビルド方法】
//   Linux(Raspberry Pi):
//     g++ test_client.cpp -std=c++17 `pkg-config --cflags --libs opencv4` -lwiringPi -O2 -o client
//
// 【注意】
//   cvui.h をこのファイルと同じフォルダに置いてください
// ============================================================================
// ============================================================================
// 必要なライブラリの読み込み
// ============================================================================
#include <iostream>      // cout, cerr（画面に文字を出す）
#include <vector>        // 可変長配列（サイズを後から変えられる配列）
#include <string>        // 文字列を扱う
#include <cstdint>       // uint8_t, uint32_t など固定サイズの整数型
#include <random>        // 乱数生成（今回は使ってない）
#include <thread>        // スレッド（並列処理）
#include <chrono>        // 時間を扱う（スリープ用）
#include <cstring>       // memcpy など C言語の文字列関数
#include <mutex>         // mutex（排他制御）
#include <atomic>        // atomic（スレッド安全な共有変数）

#include <wiringPi.h>    // ラズパイのGPIO制御ライブラリ
#include <wiringPiSPI.h> // ラズパイのSPI通信ライブラリ（ADコンバータと通信）

#include <opencv2/opencv.hpp>  // OpenCV: カメラ撮影＆画像処理ライブラリ

// cvui: UIライブラリ（今回はUI機能は使わないけど、取り込んでおく）
#define CVUI_IMPLEMENTATION
#include "cvui.h"

// ===== 繧ｯ繝ｭ繧ｹ繝励Λ繝・ヨ繝輔か繝ｼ繝 繧ｽ繧ｱ繝・ヨ繝ｩ繝・ヱ =====
#ifdef _WIN32
  #include <winsock2.h>
  #include <ws2tcpip.h>
  #pragma comment(lib, "ws2_32.lib")
  using sock_t = SOCKET;
  #define CLOSESOCK closesocket
  #define SOCKERR SOCKET_ERROR
  #define INVALID_SOCK INVALID_SOCKET
  static void sleep_ms(int ms){ ::Sleep(ms); }
#else
  #include <sys/socket.h>
  #include <arpa/inet.h>
  #include <unistd.h>
  #include <netinet/in.h>
  #include <netinet/tcp.h>
  using sock_t = int;
  #define CLOSESOCK ::close
  #define SOCKERR (-1)
  #define INVALID_SOCK (-1)
  static void sleep_ms(int ms){ std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }
#endif

// ============================================================================
// サーバーへの接続先設定
// ============================================================================
static const char* SERVER_IP   = "192.168.11.8";  // サーバーのIPアドレス
static const uint16_t SERVER_PORT = 10000;          // サーバーのポート番号

// ============================================================================
// 【改善】送信画像の設定（小さくすると高速化）
// ============================================================================
static const int CAPTURE_WIDTH  = 400;
static const int CAPTURE_HEIGHT = 300;
static const int JPEG_QUALITY   = 90;

// read_mcp3002() は後ろで定義するため、ここで前方宣言
static int read_mcp3002(int channel);

// ============================================================================
// 【改善】スレッド間共有データ
// ============================================================================
static std::mutex g_frameMutex;
static cv::Mat g_latestFrame;
static std::atomic<bool> g_frameReady{ false };

static std::atomic<uint32_t> g_latestValue{ 0 };
static std::atomic<bool> g_running{ true };

// ============================================================================
// 便利な関数たち（ユーティリティ）
// ============================================================================

/**
 * 【データ送信関数】
 * 指定したバイト数を全部送るまで繰り返す
 * 戻り値: 成功=true, 失敗=false
 */
static bool send_all(sock_t s, const uint8_t* data, size_t len) {
  size_t sent = 0;
  while (sent < len) {
    int n = ::send(s, reinterpret_cast<const char*>(data + sent),
                   static_cast<int>(len - sent), 0);
    if (n == SOCKERR) return false;
    sent += static_cast<size_t>(n);
  }
  return true;
}

/**
 * 【データ受信関数】
 * 指定したバイト数を全部受け取るまで繰り返す
 * 戻り値: 成功=true, 失敗（切断・エラー）=false
 */
static bool recv_full(sock_t s, uint8_t* buf, size_t len) {
  size_t got = 0;  // 今まで受け取ったバイト数
  while (got < len) {
    // 残りのデータを受信
    int n = ::recv(s, reinterpret_cast<char*>(buf + got),
                   static_cast<int>(len - got), 0);
    if (n <= 0) return false; // 0以下 = 切断 or エラー
    got += static_cast<size_t>(n);
  }
  return true;
}

/**
 * 【サーバー接続関数】
 * サーバーに接続できるまで、1秒おきに何度でも再試行する
 * 戻り値: 接続成功したソケット
 */
static sock_t connect_to_server_loop(const char* ip, uint16_t port) {
#ifdef _WIN32
  // Windows用: Winsock初期化
  WSADATA wsa;
  WSAStartup(MAKEWORD(2,2), &wsa);
#endif

  // 接続できるまで無限ループ（終了要求があれば抜ける）
  while (g_running) {
    // ソケット作成（電話機を用意するイメージ）
    sock_t s = ::socket(AF_INET, SOCK_STREAM, 0);
    if (s == INVALID_SOCK) {
      std::cerr << "[socket] create failed. retry...\n";
      sleep_ms(1000);  // 1秒待ってリトライ
      continue;
    }

    // 接続先の住所（IPアドレス＋ポート番号）を設定
    sockaddr_in addr{};
    addr.sin_family = AF_INET;       // IPv4を使う
    addr.sin_port = htons(port);     // ポート番号（ネットワークバイトオーダーに変換）
#ifdef _WIN32
    inet_pton(AF_INET, ip, &addr.sin_addr);  // IPアドレス文字列を数値に変換
#else
    addr.sin_addr.s_addr = inet_addr(ip);    // Linux版のIPアドレス変換
#endif

    // サーバーに接続を試みる（電話をかける）
    if (::connect(s, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0) {
      // TCP_NODELAY（Nagle無効化）で遅延削減
      {
        int flag = 1;
#ifdef _WIN32
        if (setsockopt(s, IPPROTO_TCP, TCP_NODELAY, (const char*)&flag, sizeof(flag)) == SOCKERR) {
          std::cerr << "[socket] setsockopt(TCP_NODELAY) failed.\n";
        }
#else
        if (setsockopt(s, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag)) == SOCKERR) {
          std::cerr << "[socket] setsockopt(TCP_NODELAY) failed.\n";
        }
#endif
      }

      // 接続成功！
      std::cout << "[socket] connected to " << ip << ":" << port << "\n";
      return s;  // 接続済みソケットを返す
    }

    // 接続失敗 → 1秒待ってリトライ
    std::cerr << "[socket] connect failed. retry...\n";
    CLOSESOCK(s);
    sleep_ms(1000);
  }

  return INVALID_SOCK;
}

/**
 * 【ACK（確認応答）待ち関数】
 * サーバーから ACK(0x06) が来るのを待つ
 * 戻り値: ACKが来た=true, それ以外=false
 */
static bool wait_ack(sock_t s) {
  uint8_t ack = 0;
  if (!recv_full(s, &ack, 1)) return false;  // 1バイト受信
  return ack == 0x06;  // 0x06 = ACK（了解の意味）
}

/**
 * 【カメラ画像をJPEGで取得する関数】
 * カメラから1フレーム読み取り、JPEG形式に変換して返す
 * 
 * 引数:
 *   cap: カメラオブジェクト
 *   w, h: 出力する画像のサイズ（幅, 高さ）
 * 
 * 戻り値: JPEGデータ（バイト配列）、失敗時は空
 */
static std::vector<uint8_t> encode_jpeg_or_empty(const cv::Mat& frame) {
  std::vector<uint8_t> jpg;
  std::vector<int> params = { cv::IMWRITE_JPEG_QUALITY, JPEG_QUALITY };
  if (!cv::imencode(".jpg", frame, jpg, params)) return {};
  return jpg;
}

// 【改善】別スレッドで取得済みの最新フレームをJPEG化（待ち時間を減らす）
static std::vector<uint8_t> grab_jpeg_fast() {
  cv::Mat frame;
  {
    std::lock_guard<std::mutex> lock(g_frameMutex);
    if (g_latestFrame.empty()) {
      return {};
    }
    frame = g_latestFrame.clone();
  }
  return encode_jpeg_or_empty(frame);
}

// ============================================================================
// 【改善】カメラキャプチャスレッド
// カメラから常時フレームを取得し、最新フレームをバッファに保持
// ============================================================================
static void cameraThread(cv::VideoCapture& cap) {
  cv::Mat frame;
  cv::Mat resized;

  while (g_running) {
    if (!cap.read(frame) || frame.empty()) {
      sleep_ms(10);
      continue;
    }

    if (frame.cols != CAPTURE_WIDTH || frame.rows != CAPTURE_HEIGHT) {
      cv::resize(frame, resized, cv::Size(CAPTURE_WIDTH, CAPTURE_HEIGHT), 0, 0, cv::INTER_LINEAR);
    } else {
      resized = frame;
    }

    {
      std::lock_guard<std::mutex> lock(g_frameMutex);
      g_latestFrame = resized.clone();
      g_frameReady = true;
    }
  }
}

// ============================================================================
// 【改善】ADC読み取りスレッド（常時更新）
// ============================================================================
static void adcThread() {
  while (g_running) {
    uint32_t value = static_cast<uint32_t>(read_mcp3002(0));
    g_latestValue.store(value);
    sleep_ms(10);
  }
}

/**
 * 【MCP3002からアナログ値を読み取る関数】
 * MCP3002はADコンバータIC（アナログ→デジタル変換）
 * センサーの電圧値をデジタル値（0〜1023）に変換して返す
 * 
 * 引数: channel = 読み取るチャンネル（0 または 1）
 * 戻り値: 10ビットのデジタル値（0〜1023）
 * 
 * ＜MCP3002とは？＞
 *   アナログセンサーの電圧を数値に変えるIC。
 *   0V → 0、3.3V → 1023 になる。
 */
static int read_mcp3002(int channel) {
    unsigned char data[2];  // SPI通信用のバッファ

    // MCP3002への送信データを作成
    // 0b01100000: スタートビット + シングルエンド + MSBF
    // channel << 4: 使うチャンネル（CH0=0x00, CH1=0x10）
    data[0] = 0b01100000 | (channel << 4);
    data[1] = 0x00;

    // SPI通信で送受信（送ると同時に結果が返ってくる）
    wiringPiSPIDataRW(0, data, 2);

    // 受信データから10ビット値を取り出す
    // data[0]の下位2ビット + data[1]の8ビット = 10ビット
    return ((data[0] & 0x03) << 8) | data[1];
}

// ============================================================================
// メイン関数（プログラムのスタート地点）
// ============================================================================
int main() {
  // --------------------------------------------------------------------------
  // カメラの初期化（640x480の解像度で撮影）
  // --------------------------------------------------------------------------
  cv::VideoCapture cap(0);  // カメラを開く（0 = デフォルトカメラ）
  if (!cap.isOpened()) {
    // カメラが開けなかったらエラーで終了
    std::cerr << "[camera] cannot open camera.\n";
    return 1;
  }
  // カメラの解像度を設定
  cap.set(cv::CAP_PROP_FRAME_WIDTH,  CAPTURE_WIDTH);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, CAPTURE_HEIGHT);
  // 古いフレームを溜めない（対応しているカメラ/ドライバのみ有効）
  cap.set(cv::CAP_PROP_BUFFERSIZE, 1);

  // --------------------------------------------------------------------------
  // SPI通信の初期化（ADコンバータ MCP3002 と通信するため）
  // --------------------------------------------------------------------------
  if (wiringPiSPISetup(0, 100000) < 0) {  // チャンネル0、100kHz
      perror("wiringPiSPISetup");  // 失敗したらエラー表示
      return 1;
  }  

  // --------------------------------------------------------------------------
  // 変数の初期化
  // --------------------------------------------------------------------------
  // cvuiは今回は使わないけど、必要になったらここで初期化
  // cvui::init("status");  // ステータスウィンドウを作る場合

  // 最後に読み取った電圧値を保存する変数
  // （以前は乱数でテストしてたが、今は実際のセンサー値を使う）
  // std::mt19937 rng{std::random_device{}()};
  // std::uniform_int_distribution<int> dist(0, 5000);
  uint32_t last_value = 0;  // 0で初期化

  // --------------------------------------------------------------------------
  // 【改善】別スレッド起動（カメラ/ADCを常時更新）
  // --------------------------------------------------------------------------
  std::thread camThread(cameraThread, std::ref(cap));
  std::thread adcReadThread(adcThread);

  // カメラの準備待ち（最初のフレームが来るまで少し待つ）
  for (int i = 0; i < 50 && !g_frameReady; ++i) {
    sleep_ms(100);
  }

  // --------------------------------------------------------------------------
  // サーバーへ接続（接続できるまで何度でもリトライ）
  // --------------------------------------------------------------------------
  sock_t sock = connect_to_server_loop(SERVER_IP, SERVER_PORT);
  if (sock == INVALID_SOCK) {
    g_running = false;
    if (camThread.joinable()) camThread.join();
    if (adcReadThread.joinable()) adcReadThread.join();
    return 1;
  }

  // --------------------------------------------------------------------------
  // メインループ：サーバーからのコマンドを待ち続ける
  // --------------------------------------------------------------------------
  while (g_running) {
    // サーバーから1バイトのコマンドを受信
    uint8_t cmd = 0;
    if (!recv_full(sock, &cmd, 1)) {
      // 受信失敗 = 切断されたので再接続
      std::cerr << "[socket] disconnected. reconnecting...\n";
      CLOSESOCK(sock);
      sock = connect_to_server_loop(SERVER_IP, SERVER_PORT);
      if (sock == INVALID_SOCK) break;
      continue;
    }

    // ==========================================================================
    // コマンド 0x01: セッション開始
    // ==========================================================================
    if (cmd == 0x01) {
      // ACK（了解）を返す
      uint8_t ack = 0x06;
      if (!send_all(sock, &ack, 1)) { goto reconnect; }

      // セッション中のループ（flag=false になるまで繰り返す）
      bool flag = true;
      while (flag) {
        // セッション内のコマンドを受信
        uint8_t inner = 0;
        if (!recv_full(sock, &inner, 1)) {
          std::cerr << "[socket] disconnected during session. reconnecting...\n";
          goto reconnect;
        }

        switch (inner) {
          // ----------------------------------------------------------------
          // コマンド 0x02: 「画像をください」
          // カメラで撮影してJPEGでサーバーに送る
          // ----------------------------------------------------------------
          case 0x02: {
            // 【改善】最新フレームをJPEG化（待ちをなくす）
            auto jpg = grab_jpeg_fast();
            if (jpg.empty()) {
              // サーバ側はサイズ0を拒否するので、ダミー画像を送る
              cv::Mat dummy(CAPTURE_HEIGHT, CAPTURE_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));
              jpg = encode_jpeg_or_empty(dummy);
            }

            // 【改善】ADC値は別スレッドで更新済みの値を即取得
            last_value = g_latestValue.load();

            // 送信データを組み立てる
            // フォーマット: [ACK(1)] [0x11(1)] [サイズ(4)] [JPEGデータ]
            uint32_t sz = static_cast<uint32_t>(jpg.size());
#ifdef _WIN32
            uint32_t nsz = htonl(sz);
            uint32_t nval = htonl(last_value);
#else
            uint32_t nsz = htonl(sz);
            uint32_t nval = htonl(last_value);
#endif
            (void)nval;  // 画像パケットではnvalは使わない（警告消し）

            // 送信用バッファを作成
            std::vector<uint8_t> out;
            out.reserve(1 + 1 + 4 + sz);  // 必要なサイズを予約
            out.push_back(0x06);  // ACK（了解しました）
            out.push_back(0x11);  // 0x11 = 「これは画像データだよ」という印

            // JPEGサイズを4バイトで追加（ネットワークバイトオーダー）
            uint8_t* p = reinterpret_cast<uint8_t*>(&nsz);
            out.insert(out.end(), p, p + 4);

            // JPEGデータ本体を追加
            out.insert(out.end(), jpg.begin(), jpg.end());

            // データ送信 → サーバーからACKを待つ
            if (!send_all(sock, out.data(), out.size())) { goto reconnect; }
            if (!wait_ack(sock)) { std::cerr << "[proto] ACK wait failed after image.\n"; goto reconnect; }
            break;
          }

          // ----------------------------------------------------------------
          // コマンド 0x03: 「電圧値をください」
          // ADコンバータで読み取った値をサーバーに送る
          // ----------------------------------------------------------------
          case 0x03: {
            // 送信データを組み立てる
            // フォーマット: [ACK(1)] [0x12(1)] [電圧値(4)]
#ifdef _WIN32
            uint32_t nval = htonl(last_value);  // ネットワークバイトオーダーに変換
#else
            uint32_t nval = htonl(last_value);
#endif
            std::vector<uint8_t> out;
            out.reserve(1 + 1 + 4);
            out.push_back(0x06);  // ACK
            out.push_back(0x12);  // 0x12 = 「これは電圧値データだよ」という印
            uint8_t* pv = reinterpret_cast<uint8_t*>(&nval);
            out.insert(out.end(), pv, pv + 4);

            if (!send_all(sock, out.data(), out.size())) { goto reconnect; }
            if (!wait_ack(sock)) { std::cerr << "[proto] ACK wait failed after value.\n"; goto reconnect; }
            break;
          }
          case 0x04: {
            // ----------------------------------------------------------------
            // コマンド 0x04: 「セッション終了」
            // セッション内ループを抜けて、コマンド待ちに戻る
            // ----------------------------------------------------------------
            uint8_t ack = 0x06;
            if (!send_all(sock, &ack, 1)) { goto reconnect; }
            flag = false;  // セッションループを抜ける
            break;
          }
          case 0x05: {
            // ----------------------------------------------------------------
            // コマンド 0x05: 「プログラムを終了して」
            // セッション中にシャットダウンが来た場合
            // ----------------------------------------------------------------
            uint8_t ack = 0x06;
            send_all(sock, &ack, 1);
            std::cout << "[proto] shutdown received in-session.\n";
            CLOSESOCK(sock);
            g_running = false;
#ifdef _WIN32
            WSACleanup();
#endif
            flag = false;
            goto cleanup;
          }
          default:
            // ----------------------------------------------------------------
            // それ以外のコマンド（未定義）は無視
            // ----------------------------------------------------------------
            break;
        }
      }
    }
    else if (cmd == 0x05) {
      // 繧ｷ繝｣繝・ヨ繝繧ｦ繝ｳ
      uint8_t ack = 0x06;
      send_all(sock, &ack, 1);
      std::cout << "[proto] shutdown received.\n";
      g_running = false;
      break;
    }
    else {
      // 未定義コマンドは無視
    }
    continue;

reconnect:
    std::cerr << "[socket] error. reconnecting...\n";
    CLOSESOCK(sock);
    sock = connect_to_server_loop(SERVER_IP, SERVER_PORT);
    if (sock == INVALID_SOCK) break;
  }

cleanup:
  if (sock != INVALID_SOCK) CLOSESOCK(sock);

  g_running = false;
  if (camThread.joinable()) camThread.join();
  if (adcReadThread.joinable()) adcReadThread.join();

#ifdef _WIN32
  WSACleanup();
#endif
  return 0;
}

