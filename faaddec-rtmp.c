/**
 * faaddec.c
 * use faad library to decode AAC, only can decode frame with ADTS head 
 */
#include <stdio.h>
#include <memory.h>
#include "faad.h"
#include <librtmp/rtmp.h>
#include <stdlib.h>

  
#define FRAME_MAX_LEN 1024*5 
#define BUFFER_MAX_LEN 3*1024*1024

// ����  gcc -o faaddec-rtmp faaddec-rtmp.c -l faad -l rtmp

void show_usage()
{
    printf("usage\nfaaddec src_file dst_file\n");
}

/**
 * fetch one ADTS frame
 */
int get_one_ADTS_frame(unsigned char* buffer, size_t buf_size, unsigned char* data ,size_t* data_size)
{
    size_t size = 0;

    if(!buffer || !data || !data_size )
    {
        return -1;
    }

    while(1)
    {
        if(buf_size  < 7 )
        {
            return -1;
        }

        if((buffer[0] == 0xff) && ((buffer[1] & 0xf0) == 0xf0) )
        {
            size |= ((buffer[3] & 0x03) <<11);     //high 2 bit
            size |= buffer[4]<<3;                //middle 8 bit
            size |= ((buffer[5] & 0xe0)>>5);        //low 3bit
            break;
        }
        --buf_size;
        ++buffer;
    }

    if(buf_size < size)
    {
        return -1;
    }

    memcpy(data, buffer, size);
    *data_size = size;
    
    return 0;
}


/* Private define ------------------------------------------------------------*/

//��Ϣ�����ֽڴ�С
#define MAX_SIZE_MSGBUF                 8

//��Ϣ���ͣ����Ӧ�Ĳ�����[]������
#define MQT_STOP                        0   //ֹͣ[��Ӧsem�����4B]
#define MQT_INPUT                       1   //֡����[frameObject�����4B]

//rtmp֡������󳤶�
#define MAX_SIZE_RTMP_PACKET_BODY       (1500 * 64)

/* Private typedef -----------------------------------------------------------*/

//rtmp��������
typedef struct
{   
    //librtmp�����ṹ
    RTMP                                rtmp;
    //librtmp���Ľṹ
    RTMPPacket                          packet;
    uint32_t                           startTime;
    //����Ƶ��(��λ��Hz)
    uint32_t                            sampleRate;
    uint8_t                             isSendSequenceHeader;
    //�Ƿ�����ͨ��������
    uint8_t                             isStreamConnectOK;
    //�Ựid
    char*                               sessionID;
    //Ŀ��url
    char                               url[128];
}RTMP_PUSH_OPERATE;

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


static uint32_t        gTimeStamp = 0;//debug
static uint32_t gPreTime = 0;

/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Extern function prototypes ------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @brief   rtmp����������������
 * @param   in  rtmpPushOpt rtmp�����������
 */
static void rtmpPushEndHandle(RTMP_PUSH_OPERATE* rtmpPushOpt)
{
    uint16_t    msgType, msgSize;
    
    
    printf("rtmpPush�߳��˳�...\n");

    if(!RTMP_IsConnected(&rtmpPushOpt->rtmp))
    {
        RTMP_Close(&rtmpPushOpt->rtmp);
    }

    free(rtmpPushOpt);
}

/**
 * @brief   ����RTMP������
 * @param   in  rtmpPushOpt rtmp�����������
 */
static int rtmpPushConnectRTMP(RTMP_PUSH_OPERATE* rtmpPushOpt)
{


    printf("rtmpPush��������...%d\n", RTMP_GetTime());//debug
    printf("%s(%d) url =%s\n", __FUNCTION__, __LINE__, rtmpPushOpt->url);
    rtmpPushOpt->isSendSequenceHeader = 0; //debug


    
    //��ʼ��librtmp
    
    printf("%s(%d) url =%s\n", __FUNCTION__, __LINE__, rtmpPushOpt->url);
    //������URL
    rtmpPushOpt->rtmp.Link.timeout = 5;
    printf("%s(%d) url =%s\n", __FUNCTION__, __LINE__, rtmpPushOpt->url);
    if(!RTMP_SetupURL(&rtmpPushOpt->rtmp,
        rtmpPushOpt->url))
    {
        printf("rtmpPush��URL[%s]�Ƿ�!",
            rtmpPushOpt->url);
        return -1;
    }
    //if unable,the AMF command would be 'play' instead of 'publish'
    printf("%s(%d) url =%s\n", __FUNCTION__, __LINE__, rtmpPushOpt->url);
	RTMP_EnableWrite(&rtmpPushOpt->rtmp);
    //������·����
    printf("%s(%d) url =%s\n", __FUNCTION__, __LINE__, rtmpPushOpt->url);
    if(!RTMP_Connect(&rtmpPushOpt->rtmp, NULL))
    {
        printf("rtmpPush��·[%s]����ʧ��!",
            rtmpPushOpt->url);
        return -1;
    }
    //����������
    printf("%s(%d)\n", __FUNCTION__, __LINE__);
    if(!RTMP_ConnectStream(&rtmpPushOpt->rtmp, 0))
    {
        printf("rtmpPush��[%s]����ʧ��!",
            rtmpPushOpt->url);
        return -1;
    }
    printf("%s(%d)\n", __FUNCTION__, __LINE__);
    if(TRUE != RTMPPacket_Alloc(&rtmpPushOpt->packet, MAX_SIZE_RTMP_PACKET_BODY))
    {
        printf("rtmpPush����packet����ʧ��!\n");
        return -1;
    }
    printf("%s(%d)\n", __FUNCTION__, __LINE__);
    RTMPPacket_Reset(&rtmpPushOpt->packet);
    rtmpPushOpt->packet.m_hasAbsTimestamp = 0;  
    rtmpPushOpt->packet.m_nChannel = 0x04;  
    rtmpPushOpt->packet.m_nInfoField2 = rtmpPushOpt->rtmp.m_stream_id;
    rtmpPushOpt->startTime = RTMP_GetTime();
    rtmpPushOpt->isStreamConnectOK = 1;
    printf("rtmpPush stream[%d, %s] connect...............\n", rtmpPushOpt->rtmp.m_stream_id,
        rtmpPushOpt->url);

    printf("%s(%d)\n", __FUNCTION__, __LINE__);
    gTimeStamp = 0;//debug
    gPreTime = 0;
    return 0;
}

/**
 * @brief   ���������һ��rtmp����
 * @param   in  rtmpPushOpt rtmp�����������
 * @param   in  frameObject �����͵�ý��֡����  //debug
 */
static int rtmpPushSendPacket(RTMP_PUSH_OPERATE* rtmpPushOpt,
                    uint8_t* header,    uint32_t sizeHeader,
                    uint8_t* payload,   uint32_t sizepayload,
                    uint32_t timeStamp)
{
    if(!RTMP_IsConnected(&rtmpPushOpt->rtmp))
    {
        printf
            ("rtmpPush����δ����!\n");
        return -1;
    }
    RTMPPacket_Reset(&rtmpPushOpt->packet);
    //���rtmp packet
    rtmpPushOpt->packet.m_headerType = RTMP_PACKET_SIZE_LARGE;
    rtmpPushOpt->packet.m_packetType = 8;//����Ƶ�����ͣ�0x8-Audio��0x9-Video��0x12-Info
    rtmpPushOpt->packet.m_nChannel = 5;//����Ƶͨ�����룺0x3-Metadata��0x4-Video��0x5-Audio
    rtmpPushOpt->packet.m_nTimeStamp = timeStamp;
    rtmpPushOpt->packet.m_hasAbsTimestamp = 0;
    rtmpPushOpt->packet.m_nInfoField2 = rtmpPushOpt->rtmp.m_stream_id;
    rtmpPushOpt->packet.m_nBodySize  = sizeHeader + sizepayload;
    if(MAX_SIZE_RTMP_PACKET_BODY < rtmpPushOpt->packet.m_nBodySize)
    {
        printf("rtmpPush֡[%d]����!", rtmpPushOpt->packet.m_nBodySize);
        printf("sizeHeader = %d, sizepayload = %d", sizeHeader, sizepayload);     
        return -1;
    }
    memcpy(rtmpPushOpt->packet.m_body, header, sizeHeader);
    memcpy(rtmpPushOpt->packet.m_body + sizeHeader, payload, sizepayload);
    if(!RTMP_SendPacket(&rtmpPushOpt->rtmp, &rtmpPushOpt->packet, 0))
    {
        printf("rtmpPush����ʧ��!\n");
        return -1;
    }

     //printf("!timeStamp= %d\n\n", timeStamp);//debug
    
    return 0;
}

/**
 * @brief   ���������һ��rtmp����ͷ
 * @param   in  rtmpPushOpt rtmp�����������
 * @param   in  sampleRate  ����Ƶ��(��λ��Hz)
 * @param   in  sampBit     ����λ��(��λ��bit)��һ��Ϊ8��16��24
 * @param   in  channel     ��������һ��Ϊ1��2  //debug
 */
static int rtmpPushSendHeader(RTMP_PUSH_OPERATE* rtmpPushOpt,
                    uint32_t sampleRate, uint8_t sampBit, uint8_t channel,
                    uint32_t timeStamp)
{
    uint8_t                 audioSpecificConfig[2];
/* //debug    FLV_AUDIO_TAG_HEADER    audioTagHeader;

    //�������ͣ�0-��������1-������
    audioTagHeader.isStereo = ((2 =< channel) ? 1 : 0);
    //������С��0-8λ��1-16λ
    audioTagHeader.is16BitSamples = 16;
    //������ָʾ��0-5.5kHz��1-11kHz��2-22kHz��3-44kHz
    audioTagHeader.sampRateIndex  = 3;
    //��Ƶ���ݸ�ʽ
    // 0 = Linear PCM, platform endian
    // 1 = ADPCM
    // 2 = MP3
    // 3 = Linear PCM, little endian
    // 4 = Nellymoser 16 kHz mono
    // 5 = Nellymoser 8 kHz mono
    // 6 = Nellymoser
    // 7 = G.711 A-law logarithmic PCM
    // 8 = G.711 mu-law logarithmic PCM
    // 9 = reserved
    //10 = AAC
    //11 = Speex
    //14 = MP3 8 kHz
    //15 = Device-specific sound
    audioTagHeader.soundFormat    = 10;
    //AAC��������(ֻ�е�soundFormat==10ʱ��Ч)
    //0 = AAC sequence header
    //1 = AAC raw
    audioTagHeader.aacPacketType  = 0;
*/
    uint8_t header[2];//debug
    header[0] = 0xAF;//debug
    header[1] = 0x00;//debug



    //AudioSpecificConfig��ռ��2�ֽڣ���ʽΪ��xxxx xaaa aooo o111
    // AAC Profile 5bits | ������ 4bits | ������ 4bits | ���� 3bits |
    // aaa a��������
    // ooo o��������
    audioSpecificConfig[0] = (2 << 3);//AAC Profile��1-main��2-LC��3-SSR
    audioSpecificConfig[1] = 0;
    switch(sampleRate)
    {
        case 48000:
            audioSpecificConfig[0] |= (3 >> 1);
            audioSpecificConfig[1] |= ((3 & 1) << 7);
            break;
		case 44100:
            audioSpecificConfig[0] |= (4 >> 1);
            audioSpecificConfig[1] |= ((4 & 1) << 7);
            break;
		case 32000:
            audioSpecificConfig[0] |= (5 >> 1);
            audioSpecificConfig[1] |= ((5 & 1) << 7);
            break;
		case 24000:
            audioSpecificConfig[0] |= (6 >> 1);
            audioSpecificConfig[1] |= ((6 & 1) << 7);
            break;
		case 22050:
            audioSpecificConfig[0] |= (7 >> 1);
            audioSpecificConfig[1] |= ((7 & 1) << 7);
            break;
        case 16000:
            audioSpecificConfig[0] |= (0x8 >> 1);
            audioSpecificConfig[1] |= ((0x8 & 1) << 7);
            break;  
        case 12000:
            audioSpecificConfig[0] |= (0x9 >> 1);
            audioSpecificConfig[1] |= ((0x9 & 1) << 7);
            break;  
        case 11025:
            audioSpecificConfig[0] |= (0xa >> 1);
            audioSpecificConfig[1] |= ((0xa & 1) << 7);
            break;    
        case 8000:
            audioSpecificConfig[0] |= (0xb >> 1);
            audioSpecificConfig[1] |= ((0xb & 1) << 7);
            break;    
        default:
            printf
                ("rtmpPush֡������[%d]��֧��!", sampleRate);
            return -1;
    }
    switch(channel)
    {
        case 1:
        case 2:
            audioSpecificConfig[1] |= (channel << 3);
            break;
        default:
            printf
                ("rtmpPush֡������[%d]��֧��!", channel);
            return -1;
    }
    return rtmpPushSendPacket(rtmpPushOpt, header, 2,//debug  &audioTagHeader, 2,
        audioSpecificConfig, 2, timeStamp);
}

/**
 * @brief   ���������һ��rtmp���� //debug
 * @param   in  rtmpPushOpt rtmp�����������
 * @param   in  frameObject �����͵�ý��֡����
 */
static int rtmpPushSendFrame(RTMP_PUSH_OPERATE* rtmpPushOpt,
                    uint8_t *frameBufer, uint32_t size)
{
    int ret = 0;
    uint32_t        timeStamp;
    uint8_t         header[2];//debug

    //printf("frameBufer = %x, size = %d, timeStamp = %d\n", frameBufer, size, gTimeStamp);
    if(size > 0)
    {
                  // ��ʱ���Ӱ�죬�����ǳ�����
        timeStamp = gTimeStamp;
        //timeStamp = 0;
        if(0 == rtmpPushOpt->isSendSequenceHeader)
        {
            //����ͷ��Ϣ��δ����
            if(0 != rtmpPushSendHeader(rtmpPushOpt, rtmpPushOpt->sampleRate,
                16, 2, timeStamp))
            {
                return -1;
            }
            rtmpPushOpt->isSendSequenceHeader = 1;
        }
        
        header[0] = 0xAF;//debug
        header[1] = 0x01;//debug
        ret = rtmpPushSendPacket(rtmpPushOpt, header, 2,
                    frameBufer + 7, size -7 ,   timeStamp);
        //gTimeStamp += 128;
    }
    
    return ret;    
}


/**
 * @brief   ����rtmp����
 * @param   in  param   ��Ƶ�������
 * @param   in  url     ����Ŀ��URL������"http://as.comtom.cn:1443/test.txt"
 * @return  ==NULL,����ʧ�ܣ�!=NULL,�ɹ�������rtmp���Ͳ������
 * @note    1.���param!=NULL����ʾ������Ӧ��Ƶͨ�������ݲ���Ȼ�����͵�Ŀ��URL
 *          2.���param==NULL����ʾ�����Ự���ݣ�Ȼ�����͵�Ŀ��URL
 */
RTMP_PUSH_OPERATE* rtmpPushStart(char* url)
{
    RTMP_PUSH_OPERATE*  rtmpPushOpt;

    //�����������
    rtmpPushOpt = malloc(sizeof(RTMP_PUSH_OPERATE));
    if(NULL == rtmpPushOpt)
    {
        printf("�ڴ治��!\n");
        return NULL;
    }
    memset(rtmpPushOpt, 0, sizeof(RTMP_PUSH_OPERATE));
    //�洢����Ŀ��URL
    
    //��������������ӿ�
    RTMP_Init(&rtmpPushOpt->rtmp);
    strcpy(rtmpPushOpt->url, url);
    printf("rtmpPush���� [%s] ...\n", rtmpPushOpt->url);
    return rtmpPushOpt;
}


int main(int argc, char* argv[])
{
   
    static unsigned char frame[FRAME_MAX_LEN];
    static unsigned char buffer[BUFFER_MAX_LEN] = {0};

    char src_file[128] = {0};
    FILE* ifile = NULL;
    char rtmpUrl[128];

    unsigned long samplerate;
    unsigned char channels;
    NeAACDecHandle decoder = 0;

    size_t data_size = 0;
    size_t size = 0;

    NeAACDecFrameInfo frame_info;
    unsigned char* input_data = buffer;
    unsigned char* pcm_data = NULL;

    //analyse parameter
    if(argc < 3)
    {
        show_usage();
        return -1;
    }
    strcpy(src_file, argv[1]);
    strcpy(rtmpUrl, argv[2]);


    ifile = fopen(src_file, "rb");
    if(!ifile)
    {
        printf("source or destination file\n");
        return -1;
    }

     data_size = fread(buffer, 1, BUFFER_MAX_LEN, ifile);

     //open decoder
    decoder = NeAACDecOpen();    
    if(get_one_ADTS_frame(buffer, data_size, frame, &size) < 0)
    {
        return -1;
    }
    // ������صĲ���
    RTMP_PUSH_OPERATE* rtmpPushOpt;
    uint32_t duration = 0;
    uint32_t pts = 0;       // ��λ����
    uint32_t baseTime = 0;  // ��׼ʱ��
    // ��rtmp����
    //��ʼ��librtmp
    rtmpPushOpt = rtmpPushStart(rtmpUrl);
     printf("rtmpUrl %s, %s\n", rtmpUrl, rtmpPushOpt->url);
    // ������

    //initialize decoder
    NeAACDecInit(decoder, frame, size, &samplerate, &channels);

    uint32_t frameCount = 0;
    uint32_t preTime = 0;
    printf("samplerate %d, channels %d\n", samplerate, channels);
    if(rtmpPushConnectRTMP(rtmpPushOpt) != 0)
    {
         printf("rtmpPushConnectRTMP failed\n");
    }
    else
    {
        rtmpPushOpt->startTime = RTMP_GetTime();
        rtmpPushOpt->sampleRate = samplerate;
        while(get_one_ADTS_frame(input_data, data_size, frame, &size) == 0)
        {
            //printf("frame size %d\n", size);

            //decode ADTS frame
           // pcm_data = (unsigned char*)NeAACDecDecode(decoder, &frame_info, frame, size); 
            frame_info.error = 0;
            frame_info.samples = 2048;
            pcm_data = 1;
            if(frame_info.error > 0)
            {
                printf("%s\n",NeAACDecGetErrorMessage(frame_info.error));            

            }
            else if(pcm_data && frame_info.samples > 0)
            {
                /*
                printf("frame info: bytesconsumed %d, channels %d, header_type %d\
                    object_type %d, samples %d, samplerate %d\n", 
                    frame_info.bytesconsumed, 
                    frame_info.channels, frame_info.header_type, 
                    frame_info.object_type, frame_info.samples, 
                    frame_info.samplerate);
                */    

                frameCount++;
                gTimeStamp = pts;
                duration = 1000 * (frame_info.samples / 2) /  samplerate;
                //printf("duration = %d, %d-%d, pts = %d\n", duration, frame_info.samples, frameCount, pts);
                rtmpPushSendFrame(rtmpPushOpt, input_data, size);
                
                pts += duration;

                uint32_t curTime = RTMP_GetTime();
                printf("pts %u, %u, %u\n", pts, (curTime - rtmpPushOpt->startTime), curTime - preTime);
                preTime = curTime;
                //printf("(pts - duration) = %u, %u\n", (pts - duration), (curTime - rtmpPushOpt->startTime));
                while((pts - duration)  > (curTime - rtmpPushOpt->startTime))     //��ʱ�ȴ�ʱ��
                {
                     
                  usleep(5000);
                  curTime = RTMP_GetTime();
                  
                  //printf("curTime = %u, %u\n", curTime, preTime);
                }
                
                //usleep((duration-2) * 1000);
            }        
            
            data_size -= size;
            input_data += size;
            
        }    
    }
    RTMP_Close(&rtmpPushOpt->rtmp);
    if(rtmpPushOpt)
        free(rtmpPushOpt);
    NeAACDecClose(decoder);

    fclose(ifile);
    printf("finish\n");
	return 0;
}





