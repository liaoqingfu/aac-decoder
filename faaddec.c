/**
 * faaddec.c
 * use faad library to decode AAC, only can decode frame with ADTS head 
 */
#include <stdio.h>
#include <memory.h>
#include "faad.h"

#define FRAME_MAX_LEN 1024*5 
#define BUFFER_MAX_LEN 1024*1024

void show_usage()
{
    printf("usage\nfaaddec src_file dst_file");
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

int main(int argc, char* argv[])
{
    static unsigned char frame[FRAME_MAX_LEN];
    static unsigned char buffer[BUFFER_MAX_LEN] = {0};

    char src_file[128] = {0};
    char dst_file[128] = {0};
    FILE* ifile = NULL;
    FILE* ofile = NULL;

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
    sscanf(argv[1], "%s", src_file);
    sscanf(argv[2], "%s", dst_file);


    ifile = fopen(src_file, "rb");
    ofile = fopen(dst_file, "wb");
    if(!ifile || !ofile)
    {
        printf("source or destination file");
        return -1;
    }

     data_size = fread(buffer, 1, BUFFER_MAX_LEN, ifile);

     //open decoder
    decoder = NeAACDecOpen();    
    if(get_one_ADTS_frame(buffer, data_size, frame, &size) < 0)
    {
        return -1;
    }

    //initialize decoder
    NeAACDecInit(decoder, frame, size, &samplerate, &channels);
    printf("samplerate %d, channels %d\n", samplerate, channels);
    
    while(get_one_ADTS_frame(input_data, data_size, frame, &size) == 0)
    {
        printf("frame size %d\n", size);

        //decode ADTS frame
        pcm_data = (unsigned char*)NeAACDecDecode(decoder, &frame_info, frame, size); 
        
        if(frame_info.error > 0)
        {
            printf("%s\n",NeAACDecGetErrorMessage(frame_info.error));            

        }
        else if(pcm_data && frame_info.samples > 0)
        {
            printf("frame info: bytesconsumed %d, channels %d, header_type %d\
                object_type %d, samples %d, samplerate %d\n", 
                frame_info.bytesconsumed, 
                frame_info.channels, frame_info.header_type, 
                frame_info.object_type, frame_info.samples, 
                frame_info.samplerate);

            fwrite(pcm_data, 1, frame_info.samples * frame_info.channels, ofile);      //2¸öÍ¨µÀ
            fflush(ofile);
        }        
        data_size -= size;
        input_data += size;
    }    

    NeAACDecClose(decoder);

    fclose(ifile);
    fclose(ofile);

	return 0;
}


