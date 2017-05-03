

#ifndef __CVT_H__
#define __CVT_H__

#define RCV_CAP_BUF_SIZE            8

extern void * my_strcpy(void *dst, const void *src, unsigned int strlen);

extern int ftol(float f);

//extern dtol(i,d);

extern int StrToInt( char *str);

extern float StrToFloat(char* pstrfloat); 

//�޷��Ŷ�������������
//src---�����׵�ַ��len���鳤��(0~255)��up_down---0��ʾ����---1��ʾ����
extern void bubble16(unsigned short *src, unsigned char len, unsigned char up_down);
extern void bubble32(unsigned int *src, unsigned char len, unsigned char up_down);

//�������R[1..n]�н��ж��ֲ��ң��ɹ�ʱ���ؽ���λ�ã�ʧ��ʱ����-1
extern char BinSearch(unsigned short *src, unsigned char len, unsigned short key);

//�������R[1..n]�н��ж��ֲ��ң��ɹ�ʱ���ؽ���λ�ã�ʧ��ʱ����-1
extern char BinSearchDelete(unsigned short *src, unsigned char len, unsigned short key);

//���ֲ��뷨����ָ��Ԫ��,ǰ���Ǹö���δ��,���ز����λ��
extern char BinSearchInsert(unsigned short* src, unsigned char len,unsigned short key);

//Ԫ�ز����ð�ݷ�����Ȼ�󷵻�Ԫ�ص����кţ�ʧ�ܷ���-1��
char BubbleInsert(unsigned short* src, unsigned char len,unsigned short key);

//���ַ���ȡ����Ԫ�أ�ʧ�ܷ���-1
unsigned short GetBinSearchValue(unsigned short *src, unsigned char len, unsigned char index);

//��ȡ�������������ֵ
unsigned short GetBubbleMaxValue(unsigned short *src, unsigned char len );

//��ȡ�������������ֵ������
unsigned char GetBubbleMaxIndex(unsigned short *src, unsigned char len );

//----------------------------------------------------------------------------
//�������ƣ�rms(Root Mean Square)
//��ڲ�����unsigned int* pdata	ָ��Դ���ݱ��ָ��
//			unsigned char N	Ԫ�صĸ���
//���ز���: 0		����Ԫ�ؾ��Ϸ�
//	    0xff	û�кϷ�Ԫ��
//	    float       result    
//��������: �ú���ʵ�ֶ�Ԫ���������ֵ	
//----------------------------------------------------------------------------

extern float rms(unsigned int* pdata,unsigned char N,unsigned char* err);

//----------------------------------------------------------------------------
//�������ƣ�average
//��ڲ�����unsigned int* pdata	ָ��Դ���ݱ��ָ��
//			unsigned char N Ԫ�صĸ���	
//���ز���: 0		����Ԫ�ؾ��Ϸ�
//	    0xff	û�кϷ�Ԫ��
//	    float       result    
//��������: �ú���ʵ�ֶ�Ԫ�������㾫��Ҫ���ȥ�����ֵ����Сֵ��ƽ��ֵ	
//----------------------------------------------------------------------------
//extern OS_MEM *pMem_Comm;

extern float average(unsigned int* pdata_src,unsigned char N,unsigned char* err);

#endif