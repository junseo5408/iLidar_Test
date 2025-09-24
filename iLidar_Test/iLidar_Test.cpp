#include <thread>
#include <stdio.h>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <iostream>

#include <winsock2.h>
#include <ws2tcpip.h>

// Include iLiDAR library
#include "../src/ilidar.hpp"
using namespace std;

// 글로벌 데이터 버퍼
static iTFS::img_t lidar_img_data[iTFS::max_device];

// 동기화 변수
static std::condition_variable lidar_cv;
static std::mutex lidar_cv_mutex;
static std::queue<int> lidar_q;

// LiDAR 데이터 핸들러 (데이터 수신 시 호출됨)
static void lidar_data_handler(iTFS::device_t* device) {
    // 데이터 복사
    memcpy((void*)&lidar_img_data[device->idx].img,
        (const void*)device->data.img,
        sizeof(device->data.img));

    // 메인 루프에 알림
    int idx = device->idx;
    std::lock_guard<std::mutex> lk(lidar_cv_mutex);
    lidar_q.push(idx);
    lidar_cv.notify_one();
}

// 상태/정보 패킷 핸들러 (이번 단순 출력 예제에선 로그만)
static void status_packet_handler(iTFS::device_t* device) {
    printf("[STATUS] device %d frame %d\n", device->idx, device->status.capture_frame);
}

static void info_packet_handler(iTFS::device_t* device) {
    printf("[INFO] LiDAR #%d SN %d ready\n", device->idx, device->info.sensor_sn);
}


void sendSignalToRaspberryPI(const char* ip, int port, const char* msg) {
    WSADATA wsaData;
    WSAStartup(MAKEWORD(2, 2), &wsaData);

    SOCKET sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock == INVALID_SOCKET) {
        std::cerr << "Socket creation failed\n";
        return;
    }
    sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    inet_pton(AF_INET, ip, &addr.sin_addr);

    sendto(sock, msg, strlen(msg), 0, (sockaddr*)&addr, sizeof(addr));

    closesocket(sock);
    WSACleanup();
} 

int main(int argc, char* argv[]) {
    int port = 8080;

    // LiDAR 객체 생성
    iTFS::LiDAR* ilidar = new iTFS::LiDAR(
        lidar_data_handler,
        status_packet_handler,
        info_packet_handler
    );

    // 드라이버 준비 대기
    while (ilidar->Ready() != true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    printf("[MESSAGE] LiDAR is ready.\n");

    // 메인 루프: 포인트 출력
    while (true) {
        std::unique_lock<std::mutex> lk(lidar_cv_mutex);
        lidar_cv.wait(lk, [] { return !lidar_q.empty(); });
        int idx = lidar_q.front();
        lidar_q.pop();
        lk.unlock();

        // 첫 번째 몇 개 포인트만 출력 (예: [행0][열0~9])
        printf("[FRAME %d] First 10 points: ", lidar_img_data[idx].frame);
        for (int i = 0; i < 10; i++) {
            printf("%d ", lidar_img_data[idx].img[0][i]);
        }
        printf("\n");

        if (lidar_img_data[idx].img[0][0] < 1000) { // 1m 이내 물체 감지
            sendSignalToRaspberryPI("192.168.0.50", port, "LED_ON");
        }
        else {
            sendSignalToRaspberryPI("192.168.0.50", port, "LED_OFF");
        }
    }

    delete ilidar;
    return 0;
}
