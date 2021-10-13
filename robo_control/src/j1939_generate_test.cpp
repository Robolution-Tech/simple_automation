#ifdef WIN32 // for windows
#   include <windows.h>
#   include <process.h>
#   include <stdio.h> #   include <time.h>
#   include "controlcan.h"
#   pragma comment(lib, "controlcan.lib")
#   define msleep(ms)  Sleep(ms)
typedef HANDLE pthread_t;
#else // for linux
#   include "j1939_generate.h"
//#   include <stdio.h>
//#   include <stdlib.h>
//#   include <string.h>
//#   include <strings.h>
//#   include <sstream>
//#   include <unistd.h>
//#   include <sys/types.h>
//#   include <sys/stat.h>
//#   include <fcntl.h>
//#   include <pthread.h>
//#   include "controlcan.h"
//#   define msleep(ms)  usleep((ms)*1000)
//#   define min(a,b)  (((a) < (b)) ? (a) : (b))
#endif


namespace CanTransmission{

#define MAX_CHANNELS  4
#define CHECK_POINT  200
#define RX_WAIT_TIME  100
#define RX_BUFF_SIZE  1000

unsigned gDevType = 3;
unsigned gDevIdx = 0;
unsigned gChMask = 0;
unsigned gBaud = 0;
unsigned gTxType = 0;
unsigned gTxSleep = 0;
unsigned gTxFrames = 3;
unsigned gTxCount = 0;

unsigned s2n(const char *s)
{
    unsigned l = strlen(s);
    unsigned v = 0;
    unsigned h = (l > 2 && s[0] == '0' && (s[1] == 'x' || s[1] == 'X'));
    unsigned char c;
    unsigned char t;
    if (!h) return atoi(s);
    if (l > 10) return 0;
    for (s += 2; c = *s; s++)
    {
        if (c >= 'A' && c <= 'F') c += 32;
        if (c >= '0' && c <= '9') t = c - '0';
        else if (c >= 'a' && c <= 'f') t = c - 'a' + 10;
        else return 0;
        v = (v << 4) | t;
    }
    return v;
}

void generate_frame(VCI_CAN_OBJ *can)
{
    memset(can, 0, sizeof(VCI_CAN_OBJ));
    can->SendType = gTxType;
    can->DataLen = 8; // data length: 8
    unsigned i;
    can->Data[0] = 0x00;
    can->Data[1] = 0x01;
    can->Data[2] = 0x02;
    can->Data[3] = 0x03;
    can->Data[4] = 0x04;
    can->Data[5] = 0x05;
    can->Data[6] = 0x06;
    can->Data[7] = 0x07;

    can->ExternFlag = 1; // extern ID frame format
    can->ID = 0xff2a80; // id: bit22~bit28 == bit0~bit7
}

int verify_frame(VCI_CAN_OBJ *can)
{
    if (can->DataLen > 8) return 0; // error: data length
    unsigned bcc = 0;
    unsigned i;
    for (i = 0; i < can->DataLen; i++)
        bcc ^= can->Data[i];
    if ((can->ID & 0xff) != bcc) return 0; // error: data checksum
    if (((can->ID >> 8) & 7) != (can->DataLen - 1)) return 0; // error: data length
    if (!can->ExternFlag) return 1; // std-frame ok
    if (((can->ID >> 11) & 0x7ff) != (can->ID & 0x7ff)) return 0; // error: frame id
    if (((can->ID >> 22) & 0x7f) != (can->ID & 0x7f)) return 0; // error: frame id
    return 1; // ext-frame ok
}

typedef struct {
    unsigned channel; // channel index, 0~3
    unsigned stop; // stop RX-thread
    unsigned total; // total received
    unsigned error; // error(s) detected
} RX_CTX;

#ifdef WIN32
unsigned __stdcall rx_thread(void *data)
#else
void * rx_thread(void *data)
#endif
{
    RX_CTX *ctx = (RX_CTX *)data;
    ctx->total = 0; // reset counter

    VCI_CAN_OBJ can[RX_BUFF_SIZE]; // buffer
    int cnt; // current received
    int i;

    unsigned check_point = 0;
    while (!ctx->stop && !ctx->error)
    {
        cnt = VCI_Receive(gDevType, gDevIdx, ctx->channel, can, RX_BUFF_SIZE, RX_WAIT_TIME);
        if (!cnt)
            continue;

        for (i = 0; i < cnt; i++) {
            if (verify_frame(&can[i]))
                continue;
            printf("CAN%d: verify_frame() failed\n", ctx->channel);
            ctx->error = 1;
            break;
        }
        if (ctx->error) break;

        ctx->total += cnt;
        if (ctx->total / CHECK_POINT >= check_point) {
            printf("CAN%d: %d frames received & verified\n", ctx->channel, ctx->total);
            check_point++;
        }
    }

    printf("CAN%d RX thread terminated, %d frames received & verified: %s\n",
        ctx->channel, ctx->total, ctx->error ? "error(s) detected" : "no error");

#ifdef WIN32
    _endthreadex(0);
    return 0;
#else
    pthread_exit(0);
#endif
}

int j1939test(int channel_num = 0)
{
    // ----- init & start -------------------------------------------------

    VCI_INIT_CONFIG config;
    config.AccCode = 0;
    config.AccMask = 0xffffffff;
    config.Filter = 1;
    config.Mode = 0;
    config.Timing0 = gBaud & 0xff;
    config.Timing1 = gBaud >> 8;
    unsigned gBaud = 0x1c01; // 250kHz
    unsigned gChMask = 3; // 3 => 11 means both channel works


    int i, j;

    if (!VCI_InitCAN(gDevType, gDevIdx, channel_num, &config))
    {
        printf("VCI_InitCAN(%d) failed\n", channel_num);
        return 0;
    }
    printf("VCI_InitCAN(%d) succeeded\n", channel_num);

    if (!VCI_StartCAN(gDevType, gDevIdx, channel_num))
    {
        printf("VCI_StartCAN(%d) failed\n", channel_num);
        return 0;
    }
    printf("VCI_StartCAN(%d) succeeded\n", channel_num);
    

    // ----- RX-timeout test ----------------------------------------------

    VCI_CAN_OBJ can;

    // ----- create RX-threads --------------------------------------------
    // ----- wait --------------------------------------------------------

    printf("<ENTER> to start TX: %d*%d frames/channel, baud: t0=0x%02x, t1=0x%02x...\n",
        gTxFrames, gTxCount, config.Timing0, config.Timing1);
    getchar();

    // ----- start transmit -----------------------------------------------

    VCI_CAN_OBJ *buff = (VCI_CAN_OBJ *)malloc(sizeof(VCI_CAN_OBJ) * gTxFrames);
    int err = 0;
    unsigned tx;
    for (tx = 0; !err && tx < gTxCount; tx++)
    {
        if ((gChMask & (1 << channel_num)) == 0) continue;

        for (j = 0; j < gTxFrames; j++)
        	generate_frame(&buff[j]);
        if (gTxFrames != VCI_Transmit(gDevType, gDevIdx, channel_num, &buff[0], gTxFrames))
        {
            printf("CAN%d TX failed: ID=%08x\n", channel_num, can.ID);
            err = 1;
            break;
        }
        if (gTxSleep) msleep(gTxSleep);
    }
    free(buff);
    return 1;
}

unsigned char* itoa(int value,unsigned char* result, int base) {

    // check that the base if valid
    if (base < 2 || base > 36) { *result = '\0'; return result; }
    unsigned char* ptr = result, *ptr1 = result, tmp_char;
    
    int tmp_value;
    
    do{
        tmp_value = value;
        value /=base;
        *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
    }while(value);
    
   // // Apply negative sign
   // if (tmp_value < 0) *ptr++ = '-';
   // *ptr-- = '\0';
   // while(ptr1 < ptr) {
   //     tmp_char = *ptr;
   //     *ptr--= *ptr1;
   //     *ptr1++ = tmp_char;
   // }
    return result;

}

void my_itoa(int value, unsigned char* buf, int base){
    
    int i = 32;
    
    
    for(; value && i ; --i, value /= base) buf = "0123456789abcdef"[value % base] + buf;
    
}



CAN_Dev::CAN_Dev():gDevType_(3), gDevIdx_(0), gChMask_(3),gBaud_(0x1c01),gTxType_(0)
                  ,gTxSleep_(0),gTxFrames_(1),gTxCount_(1){ 
            if (!VCI_OpenDevice(gDevType_, gDevIdx_, 0)) {
                printf("VCI_OpenDevice failed\n");
            }

            //init
            config_.AccCode = 0;
            config_.AccMask = 0xffffffff;
            config_.Filter = 1;
            config_.Mode = 0;
            config_.Timing0 = gBaud_ & 0xff;
            config_.Timing1 = gBaud_ >> 8;
                      
            show_param();
            printf("here we assume channel_num = device id, might be wrong, please test!\n");
            if (!VCI_InitCAN(gDevType_,gDevIdx_, gDevIdx_, &config_))
            {
                printf("VCI_InitCAN(%d) failed\n", gDevIdx_);
            }
            printf("VCI_InitCAN(%d) succeeded\n", gDevIdx_);
       
            if (!VCI_StartCAN(gDevType_, gDevIdx_, gDevIdx_))
            {
                printf("VCI_StartCAN(%d) failed\n", gDevIdx_);
            }
            printf("VCI_StartCAN(%d) succeeded\n", gDevIdx_);


                  }

CAN_Dev::CAN_Dev(const std::string configFile):gDevIdx_(0),gBaud_(0x1c01),gTxSleep_(0),gTxFrames_(1),gTxCount_(1){ 
            
            
            //parse config
            config_parse(configFile);
            
            if (!VCI_OpenDevice(gDevType_, gDevIdx_, 0)) {
                printf("VCI_OpenDevice failed\n");
            }

            //init
            config_.AccCode = 0;
            config_.AccMask = 0xffffffff;
            config_.Filter = 1;
            config_.Mode = 0;
            config_.Timing0 = gBaud_ & 0xff;
            config_.Timing1 = gBaud_ >> 8;
                      
            show_param();
            printf("here we assume channel_num = device id, might be wrong, please test!\n");
            if (!VCI_InitCAN(gDevType_,gDevIdx_, gDevIdx_, &config_))
            {
                printf("VCI_InitCAN(%d) failed\n", gDevIdx_);
            }
            printf("VCI_InitCAN(%d) succeeded\n", gDevIdx_);
       
            if (!VCI_StartCAN(gDevType_, gDevIdx_, gDevIdx_))
            {
                printf("VCI_StartCAN(%d) failed\n", gDevIdx_);
            }
            printf("VCI_StartCAN(%d) succeeded\n", gDevIdx_);


                  }

CAN_Dev::CAN_Dev(unsigned gDevType, unsigned gDevIdx, unsigned gBaud):
            gDevType_(gDevType), gDevIdx_(gDevIdx), gChMask_(3),gBaud_(gBaud),gTxType_(0)
            ,gTxSleep_(0),gTxFrames_(1),gTxCount_(1){
            
            if (!VCI_OpenDevice(gDevType_, gDevIdx_, 0)) {
                printf("VCI_OpenDevice failed\n");
            }

            //init
            config_.AccCode = 0;
            config_.AccMask = 0xffffffff;
            config_.Filter = 1;
            config_.Mode = 0;
            config_.Timing0 = gBaud_ & 0xff;
            config_.Timing1 = gBaud_ >> 8;
                      

            printf("here we assume channel_num = device id, might be wrong, please test!\n");
            if (!VCI_InitCAN(gDevType_,gDevIdx_, gDevIdx_, &config_))
            {
                printf("VCI_InitCAN(%d) failed\n", gDevIdx_);
            }
            printf("VCI_InitCAN(%d) succeeded\n", gDevIdx_);
       
            if (!VCI_StartCAN(gDevType_, gDevIdx_, gDevIdx_))
            {
                printf("VCI_StartCAN(%d) failed\n", gDevIdx_);
            }
            printf("VCI_StartCAN(%d) succeeded\n", gDevIdx_);


            }

        //~CAN_Dev();
        //{
        //
        //    VCI_CloseDevice(gDevType_, gDevIdx_);
        //}
void CAN_Dev::CloseDevice(){
     VCI_CloseDevice(gDevType_, gDevIdx_);
}

void CAN_Dev::config_parse(const std::string configFile){
    /**
     * @brief load configuration file
     * 
     * input: configuration file dir
     */
    printf("Reading config file ... \n");
    std::cout << configFile << std::endl;
    std::ifstream i(configFile);
    i >> configJson_;

    if (configJson_ != nullptr) {
        gDevType_ = configJson_["DevType"];
        gChMask_ = configJson_["ChMask"];
        gTxType_ = configJson_["TxType"];
        ControlInputType_ = configJson_["ControlInputType"];
        keyboard_mode_ = configJson_["keyboard_mode"];
        twist_max_speed_ = configJson_["twist_max_speed"];
        joystick_pub_flag_ = configJson_["joystick_pub_flag"];
        joystick_record_flag_ = configJson_["joystick_record_flag"];
    }
    printf("Done update parameterss \n");
}

void CAN_Dev::show_param(){
            printf("DevType=%d, DevIdx=%d, ChMask=0x%x, Baud=0x%04x, TxType=%d, TxSleep=%d, TxFrames=0x%08x(%d), TxCount=0x%08x(%d)\n",
                gDevType_, gDevIdx_, gChMask_, gBaud_, gTxType_, gTxSleep_, gTxFrames_, gTxFrames_, gTxCount_, gTxCount_);
            
        }

void CAN_Dev::generate_frame(VCI_CAN_OBJ *can,unsigned ID, BYTE* value_h)
        {
            memset(can, 0, sizeof(VCI_CAN_OBJ));
            can->SendType = gTxType_; //0-normal
            can->DataLen = 8; // data length: 8
            can->ExternFlag = 1; // extern ID frame format
            //printf("order might be reversed, double check before test on machine!!!\n");
            //can->Data[0] = value_h[1];
            //can->Data[1] = value_h[0];
            //can->Data[2] = value_h[3];
            //can->Data[3] = value_h[2];
            //can->Data[4] = value_h[5];
            //can->Data[5] = value_h[4];
            //can->Data[6] = value_h[7];
            //can->Data[7] = value_h[6];
            // printf("send value byte %0x2,%0x2,%0x2,\n ",value_h[0],value_h[2],value_h[3]);
            can->Data[0] = value_h[0];
            can->Data[1] = value_h[1];
            can->Data[2] = value_h[2];
            can->Data[3] = value_h[3];
            can->Data[4] = value_h[4];
            can->Data[5] = value_h[5];
            can->Data[6] = value_h[6];
            can->Data[7] = value_h[7];
            can->ID = ID; // id: bit22~bit28 == bit0~bit7
            //can->ID = 0xff2a80; // id: bit22~bit28 == bit0~bit7
        }

// void CAN_Dev::int_to_hex(int value, BYTE* value_h,int size_byte)
//         {
//             memcpy(value_h,&value,size_byte);//size is the byte size
//             printf("int size %d, value size %d \n",sizeof(int),sizeof(value));
//             printf("int to hex %02x, %02x, %02x, %02x,%02x, %02x, %02x, %02x\n",value_h[0],value_h[1],
//                     value_h[2],value_h[3],value_h[4],value_h[5],value_h[6],value_h[7]);
//         }

void CAN_Dev::j1939_send(int channel_num, unsigned ID, BYTE* value)
        {
        
        
            // ----- start transmit -----------------------------------------------
        
            VCI_CAN_OBJ *buff = (VCI_CAN_OBJ *)malloc(sizeof(VCI_CAN_OBJ) * gTxFrames_);
            int err = 0;
            unsigned tx;
            for (tx = 0; !err && tx < gTxCount_; tx++)
            {
                for (int j = 0; j < gTxFrames_; j++)
                	generate_frame(&buff[j], ID,value);
                    //printf("frame generated\n");
                // printf("The gDevType is %d, gDevIdx is %d, channel_num is %d, gTxFrames_ is %d \n", gDevType_, gDevIdx, channel_num, gTxFrames_);
                if (gTxFrames_ != VCI_Transmit(gDevType_, gDevIdx, channel_num, &buff[0], gTxFrames_))
                {
                    printf("CAN%d TX failed: ID=%08x\n", channel_num, ID);
                    err = 1;
                    break;
                }
                if (gTxSleep_) msleep(gTxSleep_);
            }
            free(buff);
        }

void CAN_Dev::Remote_Init(){
    printf("remote init\n");
    unsigned ID0 = 0x18eeff80; //init para?
    unsigned ID1 = 0x18ffff80; //ignition
    unsigned sleep_time = 300; //300ms
    VCI_CAN_OBJ *buff = (VCI_CAN_OBJ *)malloc(sizeof(VCI_CAN_OBJ) * 4);
    BYTE ignition0[8] = {0xff,0xff,0x20,0x10,0x00,0x34,0x00,0x00};
    BYTE ignition1[8] = {0x03,0x01,0xff,0xff,0xff,0xff,0xff,0x00};
    BYTE ignition2[8] = {0x05,0x01,0xff,0xff,0xff,0xff,0xff,0xff};
    BYTE ignition3[8] = {0x22,0x01,0xff,0xff,0xff,0x00,0xff,0xff};

    generate_frame(&buff[0], ID0,ignition0);
    generate_frame(&buff[1], ID1,ignition1);
    generate_frame(&buff[2], ID1,ignition2);
    generate_frame(&buff[3], ID1,ignition3);

    int err = 0;
    if (1 != VCI_Transmit(gDevType_, gDevIdx, 0, &buff[0], 1))
    {
        printf("CAN%d TX failed: ID=%08x\n", 0, ID0);
        err = 1;
    }

    msleep(sleep_time);
    unsigned tx;
    for (tx = 0; !err && tx < 6; tx++)
    {

        if (3 != VCI_Transmit(gDevType_, gDevIdx, 0, &buff[1], 3))
        {
            printf("CAN%d TX failed: ID=%08x\n", 0, ID1);
            err = 1;
            break;
        }
        msleep(sleep_time);
    }
}

void CAN_Dev::Ignition()
{

            printf("Ignition!");
            int ignition_frames = 3;
            unsigned sleep_time = 300; //300ms
            unsigned txcount = 25;
            unsigned ID = 0x18ffff80;
            BYTE ignition1[8] = {0x03,0x01,0x01,0x00,0xff,0xff,0xff,0x00};
            BYTE ignition2[8] = {0x05,0x01,0x00,0x00,0xff,0xff,0xff,0xff};
            BYTE ignition3[8] = {0x22,0x01,0x00,0xff,0x00,0xff,0xff,0xff};
            VCI_CAN_OBJ *buff = (VCI_CAN_OBJ *)malloc(sizeof(VCI_CAN_OBJ) * ignition_frames);
            generate_frame(&buff[0], ID,ignition1);
            generate_frame(&buff[1], ID,ignition2);
            generate_frame(&buff[2], ID,ignition3);
            int err = 0;
            unsigned tx;
            for (tx = 0; !err && tx < txcount; tx++)
            {

                //for (int j = 0; j < ignition_frames; j++)
                //	generate_frame(&buff[j], ID,value);
                //    printf("frame generated\n");
                if (ignition_frames != VCI_Transmit(gDevType_, gDevIdx, 0, &buff[0], ignition_frames))
                {
                    printf("CAN%d TX failed: ID=%08x\n", 0, ID);
                    err = 1;
                    break;
                }
                msleep(sleep_time);
            }
            free(buff);

            unsigned txcount1 = 200;
            BYTE ignition4[8] = {0x03,0x01,0x00,0x00,0xff,0xff,0xff,0x00}; 
            BYTE cart_unlock[8] = {0x05,0x01,0x01,0x01,0xff,0xff,0xff,0xff}; // 3rd byte EM, 4th byte operate
            VCI_CAN_OBJ *buff1 = (VCI_CAN_OBJ *)malloc(sizeof(VCI_CAN_OBJ) * ignition_frames);
            generate_frame(&buff[0], ID,ignition4);
            generate_frame(&buff[1], ID,cart_unlock);
            generate_frame(&buff[2], ID,ignition3);
            //for (tx = 0; !err && tx < txcount1; tx++)
            while(1)
            {

                //for (int j = 0; j < ignition_frames; j++)
                //	generate_frame(&buff[j], ID,value);
                //    printf("frame generated\n");
                if (ignition_frames != VCI_Transmit(gDevType_, gDevIdx, 0, &buff[0], ignition_frames))
                {
                    printf("CAN%d TX failed: ID=%08x\n", 0, ID);
                    err = 1;
                    break;
                }
                msleep(sleep_time);
            }
            free(buff);
            printf("engine done");

}





}
//int main(int argc, char* argv[])
//{
//        printf("test [DevType] [DevIdx] [ChMask] [Baud] [TxType] [TxSleep] [TxFrames] [TxCount]\n"
//            "    example: test 16 0 3 0x1400 2 3 10 1000\n"
//            "                  |  | | |      | | |  | 1000 times\n"
//            "                  |  | | |      | | |\n"
//            "                  |  | | |      | | |10 frames once\n"
//            "                  |  | | |      | |\n"
//            "                  |  | | |      | |tx > sleep(3ms) > tx > sleep(3ms) ....\n"
//            "                  |  | | |      |\n"
//            "                  |  | | |      |0-normal, 1-single, 2-self_test, 3-single_self_test, 4-single_no_wait....\n"
//            "                  |  | | |\n"
//            "                  |  | | |0x1400-1M, 0x1c03-125K, ....\n"
//            "                  |  | |\n"
//            "                  |  | |bit0-CAN1, bit1-CAN2, bit2-CAN3, bit3-CAN4, 3=CAN1+CAN2, 7=CAN1+CAN2+CAN3\n"
//            "                  |  |\n"
//            "                  |  |Card0\n"
//            "                  |\n"
//            "                  |4-usbcan-ii, 5-pci9820, 14-pci9840, 16-pci9820i, ....\n"
//            );
//    
//    CAN_Dev device0;
//    device0.show_param();
//    int i = 1024;
//    BYTE valuet[8];
//    //memcpy(valuet,&i,2);
//    //printf("int to hex %02x, %02x, %02x, %02x,%02x, %02x, %02x, %02x\n",valuet[0],valuet[1],
//    //                valuet[2],valuet[3],valuet[4],valuet[5],valuet[6],valuet[7]);
//    //if(valuet[1] == 0x84){
//    //    printf("equal");
//    //}
//
//    device0.int_to_hex(i,valuet,2);
//    device0.int_to_hex(i,&valuet[2],2);
//    device0.int_to_hex(i,&valuet[4],2);
//    //device0.int_to_hex(i,&valuet[6],2);
//    printf("int to hex %02x, %02x, %02x, %02x,%02x, %02x, %02x, %02x\n",valuet[0],valuet[1],
//                    valuet[2],valuet[3],valuet[4],valuet[5],valuet[6],valuet[7]);
//
//    unsigned ID = 0xcff2a80;
//    device0.j1939_send(0,ID,valuet);
//    
//
////    gDevType = 4; gDevIdx = 0;
////    gChMask = 3;
////    gBaud = 0x1c03;
////    gTxType = 0;
////    gTxSleep = 0;
////    gTxFrames = 1;
////    gTxCount = 1;
////    printf("DevType=%d, DevIdx=%d, ChMask=0x%x, Baud=0x%04x, TxType=%d, TxSleep=%d, TxFrames=0x%08x(%d), TxCount=0x%08x(%d)\n",
////        gDevType, gDevIdx, gChMask, gBaud, gTxType, gTxSleep, gTxFrames, gTxFrames, gTxCount, gTxCount);
////
////    if (!VCI_OpenDevice(gDevType, gDevIdx, 0)) {
////        printf("VCI_OpenDevice failed\n");
////        return 0;
////    }
////    printf("VCI_OpenDevice succeeded\n");
////
////    j1939test(gDevIdx);
////
//    VCI_CloseDevice(gDevType, gDevIdx);
//    printf("VCI_CloseDevice\n");
//    return 0;
//}


