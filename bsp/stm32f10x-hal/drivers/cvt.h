

#ifndef __CVT_H__
#define __CVT_H__

#define RCV_CAP_BUF_SIZE            8

extern void * my_strcpy(void *dst, const void *src, unsigned int strlen);

extern int ftol(float f);

//extern dtol(i,d);

extern int StrToInt( char *str);

extern float StrToFloat(char* pstrfloat); 

//无符号短整型数组排序
//src---数组首地址，len数组长度(0~255)，up_down---0表示升序，---1表示降序
extern void bubble16(unsigned short *src, unsigned char len, unsigned char up_down);
extern void bubble32(unsigned int *src, unsigned char len, unsigned char up_down);

//在有序表R[1..n]中进行二分查找，成功时返回结点的位置，失败时返回-1
extern int BinSearch(unsigned short *src, unsigned char len, unsigned short key);

//在有序表R[1..n]中进行二分查找，成功时返回结点的位置，失败时返回-1
extern int BinSearchDelete(unsigned short *src, unsigned char len, unsigned short key);

//二分插入法插入指定元素,前提是该队列未满,返回插入的位置
extern int BinSearchInsert(unsigned short* src, unsigned char len,unsigned short key);

//元素插入后按冒泡法排序，然后返回元素的序列号，失败返回-1。
extern int BubbleInsert(unsigned short* src, unsigned char len,unsigned short key);

//二分法获取索引元素，失败返回-1
extern int GetBinSearchValue(unsigned short *src, unsigned char len, unsigned char index);

//获取升序有序表的最大值
extern int GetBubbleMaxValue(unsigned short *src, unsigned char len );

//获取升序有序表的最大值的索引
extern int GetBubbleMaxIndex(unsigned short *src, unsigned char len );

//----------------------------------------------------------------------------
//函数名称：rms(Root Mean Square)
//入口参数：unsigned int* pdata	指向源数据表的指针
//			unsigned char N	元素的个数
//返回参数: 0		所有元素均合法
//	    0xff	没有合法元素
//	    float       result    
//功能描述: 该函数实现对元素求均方根值	
//----------------------------------------------------------------------------

extern float rms(unsigned int* pdata,unsigned char N,unsigned char* err);

//----------------------------------------------------------------------------
//函数名称：average
//入口参数：unsigned int* pdata	指向源数据表的指针
//			unsigned char N 元素的个数	
//返回参数: 0		所有元素均合法
//	    0xff	没有合法元素
//	    float       result    
//功能描述: 该函数实现对元素求满足精度要求的去除最大值和最小值的平均值	
//----------------------------------------------------------------------------
//extern OS_MEM *pMem_Comm;

extern float average(unsigned int* pdata_src,unsigned char N,unsigned char* err);

#endif
