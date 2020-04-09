#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h> 
#include <netinet/in.h>

#include "../common/fmu_messages.h"

const int sendport = 6222;
const int recvport = 6223;

int main() {
    message::data_mpu9250_t imu; // message type from fmu_messages.h
    message::command_effectors_t eff; // from fmu_messages.h

    // socket stuff
    int sendfd = -1;
    int recvfd = -1;
    struct sockaddr_in sendaddr, recvaddr;
    
    // Creating send socket file descriptor 
    if ( (sendfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
        perror("socket creation failed"); 
        exit(EXIT_FAILURE); 
    }
    printf("created socket sending\n");
      
    bzero(&sendaddr, sizeof(sendaddr)); 
    sendaddr.sin_addr.s_addr = inet_addr("127.0.0.1"); 
    sendaddr.sin_port = htons(sendport); 
    sendaddr.sin_family = AF_INET;
    
    // Connect to partner 
    if ( connect(sendfd, (struct sockaddr *)&sendaddr, sizeof(sendaddr)) < 0 ) {
        printf("\n Error : Connect Failed \n"); 
        exit(0); 
    }
    
    // Creating recv socket file descriptor 
    if ( (recvfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
        perror("socket creation failed"); 
        exit(EXIT_FAILURE); 
    }
    int flags = fcntl(recvfd, F_GETFL, 0);
    fcntl(recvfd, F_SETFL, flags | O_NONBLOCK); // make non-blocking
    printf("created socket receiving\n");
    
    bzero(&recvaddr, sizeof(recvaddr)); 
    recvaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    recvaddr.sin_port = htons(recvport); 
    recvaddr.sin_family = AF_INET;
    
    // Bind the socket with the server address 
    if ( bind(recvfd, (const struct sockaddr *)&recvaddr, sizeof(recvaddr)) < 0 ) 
    { 
        perror("bind failed"); 
        exit(EXIT_FAILURE); 
    } 
    
    while ( true ) {
        // fill in message data
        imu.AccelX_mss = 0.1;
        imu.AccelY_mss = -0.2;
        imu.AccelZ_mss = 0.3;
        imu.GyroX_rads = -0.4;
        imu.GyroY_rads = 0.5;
        imu.GyroZ_rads = -0.6;
        imu.MagX_uT = 0.7;
        imu.MagY_uT = -0.8;
        imu.MagZ_uT = 0.9;
        imu.Temperature_C = -1.0;

        // pack the structure (and compute len)
        imu.pack();

        // send message data
        if ( send(sendfd, &imu.payload, imu.len, 0) < 0 ) {
            perror("send()");
            printf("partner script not started?\n");
        } else {
            printf("sending message: %d\n", imu.len);
        }

        // read all available incomming messages
        const int MAX_BUFFER = 1024;
        char buffer[MAX_BUFFER];
        while ( true ) {
            int len = recv(recvfd, (char *)buffer, MAX_BUFFER, 0);
            if ( len > 0 ) {
                eff.unpack((uint8_t *)buffer, len);
                printf("recieved %d bytes\n", len);
                printf(" Effectors:\n  ");
                for ( int i = 0; i < message::num_effectors; i++ ) {
                    printf("%.2f ", eff.command[i]);
                }
                printf("\n");
            } else {
                break;
            }
        }

        sleep(1);               // reduce busy/wait system load
    }

    return 0;
}
