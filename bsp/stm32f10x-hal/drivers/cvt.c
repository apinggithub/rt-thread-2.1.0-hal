

#include "cvt.h"
#include <math.h>
#include <stdint.h>


void * my_strcpy(void *dst, const void *src, unsigned int strlen)
{
   if(!(dst && src)) 
     return 0;
   char * pchdst = (char*)dst;
   char * pchsrc = (char*)src;
   while(strlen--) 
     *pchdst++ = *pchsrc++;
   
   return dst;
}

//float ת int
int ftol(float f)
{ 
    int a         = *(int*)(&f);
    int sign      = (a>>31); 
    int mantissa  = (a&((1<<23)-1))|(1<<23);
    int exponent  = ((a&0x7fffffff)>>23)-127;
    int r         = ((unsigned int)(mantissa)<<8)>>(31-exponent);
    return ((r ^ (sign)) - sign ) &~ (exponent>>31);       
}

//double ת int
union luai_Cast { double l_d; long l_l; };

#define dtol(i,d)  { volatile union luai_Cast u; \
   u.l_d = (d) + 6755399441055744.0; (i) = u.l_l; }

//�ַ���ת����
int StrToInt(char *str)
{
    
    int n = 0;	
    char *p = (char*)str;
    if(*p == '-') p++;
    while('0' <= *p && *p <= '9')		
        n = n * 10 + *p++ - '0';	
    if(*str == '-')       
        n = -n;
    return n;
}
/*
//����ĳ������˰˽��ơ�ʮ���ơ�ʮ�����Ƶ��ַ�����
int StrToInt(char * str)
{ 
   int value = 0;
   int sign = 1;
   int radix;

   if(*str == '-')
   {
      sign = -1;
      str++;
   }

   if(*str == '0' && (*(str+1) == 'x' || *(str+1) == 'X'))
   {
      radix = 16;
      str += 2;
   }
   else if(*str == '0')
   {
      radix = 8;
      str++;
   }
   else
      radix = 10;
   while(*str)
   {
      if(radix == 16)
      {
        if(*str >= '0' && *str <= '9')
           value = value * radix + *str - '0';
        else
           value = value * radix + (*str | 0x20) - 'a' + 10;
      }
      else
        value = value * radix + *str - '0';
      str++;
   }
   return sign*value;
}*/

/*
void IntToStr(int value, char* str)
{
    
}*/

//�ַ���ת������
float StrToFloat(char* pstrfloat) 
{ 
    unsigned char bNegative = 0; 
    unsigned char bDec = 0; 
 
    char* pSor = 0; 
    char chByte = '0'; 
    float fInteger = 0.0; 
    float fDecimal = 0.0; 
    float fDecPower = 0.1f; 
 
 
		// check  
    if (!pstrfloat) 
    { 
        return 0.0; 
    } 
		
    // ������λ�жϣ��ж��Ƿ��Ǹ��� 
    if (pstrfloat[0] == '-') 
    {  
        bNegative = 1; 
        pSor = pstrfloat + 1; 
    } 
    else 
    { 
        bNegative = 0; 
        pSor = pstrfloat; 
    } 

    while (*pSor != '\0') 
    { 
        chByte = *pSor; 
        if (bDec) 
        { 
            // С�� 
            if (chByte >= '0' && chByte <= '9') 
            { 
                fDecimal += (chByte - '0') * fDecPower; 
                fDecPower = fDecPower * 0.1; 
            } 
            else 
            { 
                return (bNegative? -(fInteger +  fDecimal): fInteger + fDecimal); 
            } 
        } 
        else 
        { 
            // ���� 
            if (chByte >= '0' && chByte <= '9') 
            { 
                fInteger = fInteger * 10.0 + chByte - '0'; 
            } 
            else if (chByte == '.') 
            { 
                bDec = 1; 
            } 
            else 
            { 
                return (bNegative? -fInteger : fInteger); 
            } 
        } 
        pSor++; 
    } 
    return (bNegative? -(fInteger +  fDecimal): fInteger + fDecimal); 
} 


//�޷��Ŷ�������������
//src---�����׵�ַ��len���鳤��(0~255)��up_down---0��ʾ����---1��ʾ����
void bubble16(unsigned short *src, unsigned char len, unsigned char up_down)
{
   unsigned short tmp; 
   unsigned char i,j,unchange;
 
   if (up_down == 0)  //����
   {
      for (i=0; i<len; i++)
      {
         unchange = 1;
         for (j=0; j<len-1-i; j++)
         {
            if (src[j] > src[j+1])
            {
               unchange = 0;
               tmp = src[j];
               src[j] = src[j+1];
               src[j+1] = tmp;
            }
         }
         if (unchange == 1) //�����б仯����������
         break;
      }
   }
   else
   {
      for (i=0; i<len; i++)
      {
         unchange = 1;
         for (j=0; j<len-1-i; j++)
         {
            if (src[j] < src[j+1])
            {
               unchange = 0;
               tmp = src[j];
               src[j] = src[j+1];
               src[j+1] = tmp;
            }
         }
         if (unchange == 1) //�����б仯����������
            break;
      }
   }
}
//�޷��Ŷ�������������
//src---�����׵�ַ��len���鳤��(0~255)��up_down---0��ʾ����---1��ʾ����
void bubble32( unsigned int *src, unsigned char len, unsigned char up_down)
{
   uint32_t tmp; 
   unsigned char i,j,unchange;
 
   if (up_down == 0)  //����
   {
      for (i = 0; i<len; i++)
      {
         unchange = 1;
         for (j = 0; j<len-1-i; j++)
         {
            if (src[j] > src[j+1])
            {
               unchange = 0;
               tmp = src[j];
               src[j] = src[j+1];
               src[j+1] = tmp;
            }
         }
         if (unchange == 1) //�����б仯����������
         break;
      }
   }
   else
   {
      for (i=0; i<len; i++)
      {
         unchange = 1;
         for (j=0; j<len-1-i; j++)
         {
            if (src[j] < src[j+1])
            {
               unchange = 0;
               tmp = src[j];
               src[j] = src[j+1];
               src[j+1] = tmp;
            }
         }
         if (unchange == 1) //�����б仯����������
            break;
      }
   }
}
//�۰�����޷���Ԫ��
//�������R[1..n]�н��ж��ֲ��ң��ɹ�ʱ���ؽ���λ�ã�ʧ��ʱ����-1
int BinSearch(unsigned short *src, unsigned char len, unsigned short key)
{       
   unsigned char low,high, mid;//�õ�ǰ���������ϡ��½�ĳ�ֵ
 
   while((len>1)&&(src[len-1]==0)) len--;
   low = 0;
   high = len-1;
   if((key < src[low])||(key > src[high]))//Ԫ�ز��ڴα���
       return -1;
   if(key == src[low])
       return low;
   if(key == src[high])
       return high;
   while(low < high)//��ǰ��������R[low..high]�ǿ�
   {
      mid = low + ((high-low)/2); //ʹ�� (low + high) / 2 �����������������
   
      if(key == src[mid]) 
          return mid; //���ҳɹ�����
   
      if(key < src[mid])
      {
         high = mid-1; //������R[low..mid-1]�в���
         if(key == src[high])//����ϱ߽��Ԫ������Ҫ�ҵ�Ԫ��
             return high;
      }
      else
      {
         low = mid+1; //������R[mid+1..high]�в���
         if(key == src[low])//����±߽��Ԫ������Ҫ�ҵ�Ԫ��
             return low;
      }
    }
    return -1; //��low>highʱ��ʾ��������Ϊ�գ�����ʧ��
} //BinSeareh

//���ֲ��뷨����ָ��Ԫ��,ǰ���Ǹö���������һ��Ԫ��,ɾ���ɹ��󷵻���λ��
//�������R[1..n]�н��ж��ֲ��ң��ɹ�ʱ���ؽ���λ�ã�ʧ��ʱ����-1
int BinSearchDelete(unsigned short *src, unsigned char len, unsigned short key)
{       
   unsigned char i, low,high,mid;//�õ�ǰ���������ϡ��½�ĳ�ֵ
   //��ȷ������Ԫ�ر�
     while((len>=1)&&(src[len-1]==0)) len--;
     low = 0;
     high = len-1;
   
   if((key < src[low])||(key > src[high]))//Ԫ�ز��ڴα���
       return 0;
   if(key == src[low])//�����ɾ����Ԫ���ڱ�ͷ
   {
      
      for(i=low;i<len-1;i++)//���˺����Ԫ��ǰ��
      {
          src[i] = src[i+1];
      }
      src[i] = 0;
      return low;
   }
   if(key == src[high])//����ڱ�β����ֱ��ɾ��
   {
       src[high] = 0;
       return high;
   }
   while(low < high)//��ǰ��������R[low..high]�ǿ�
   {
      mid = low + ((high-low)/2); //ʹ�� (low + high) / 2 �����������������   
      if(key == src[mid]) 
      {
          for(i=mid;i<len-1;i++)//����Ԫ��ǰ��
          {
              src[i] = src[i+1];
          }
          src[i] = 0;
          return mid; //����ɾ����λ��
      }
      if(key < src[mid])
      {
         high = mid-1; //������R[low..mid-1]�в���
         if(key == src[high])//����ϱ߽��Ԫ������Ҫ�ҵ�Ԫ��
         {    
            for(i=high;i<len-1;i++)//����Ԫ��ǰ��
            {
                src[i] = src[i+1];
            }
            src[i] = 0;    
            return high;//����ɾ����λ��
         }
      }
      else
      {
         low = mid+1; //������R[mid+1..high]�в���
         if(key == src[low])//����±߽��Ԫ������Ҫ�ҵ�Ԫ��
         {
            for(i=low;i<len-1;i++)
            {
                src[i] = src[i+1];
            }
            src[i] = 0;
            return low;
         }
      }
    }
    return -1; //��low>highʱ��ʾ��������Ϊ�գ�����ʧ��
} //BinSeareh

//���ֲ��뷨����ָ��Ԫ��,ǰ���Ǹö���δ��,���ز����λ��
//���㷨���Ͻ��������⣬1~15�Ȳ������������ż��ʱ���������4����5�ĺ��棬10����11���������(��������)��
int BinSearchInsert(unsigned short* src, unsigned char len,unsigned short key)
{ 
     unsigned char  i, low, high, mid,flag;
     if(src[len-1]!=0)//������һ��Ԫ�ز��գ�˵����������
         return -1;
     //��ȷ������Ԫ�ر�
     while((len>1)&&(src[len-1]==0)) 
         len--;    
     low = 0;
     high = len-1; 

     if(key < src[low]) //����ȱ�ͷԪ��С���Ͳ��뵽��ͷ
     {
         for(i=len-1;i>low;i--)
         {
             src[i+1] = src[i];
         }
         src[i+1] = src[i];
         src[i] = key; 
         return low;
     }
     if(key > src[high])//����ȱ�βԪ�ش󣬾Ͳ��뵽��β
     {
         if(src[high]!=0)
         {
            high++;           
         }
         src[high] = key;
         return high;
     } 
     
     while (low < high) // ��R[low...high]���۰������������λ�� 
     {
                               
         mid = low + ((high-low)/2);// �ҵ��м�Ԫ��  //ʹ�� (low + high) / 2 �����������������
        
         if(key == src[mid])      //��������Ѿ����ڴ������Ԫ��
            return mid;
        
         if (key < src[mid])      // ����������Ԫ�ر��м�Ԫ��ǰС��
         {
            high = mid-1;     //�߽��ǰ��
            flag = 0;
         }
         else                    // ����������Ԫ�ر��м�Ԫ��ǰ�� 
         {
            low = mid+1;      //�ͽ�����
            flag = 1;
         }
     }
     //��low��high֮���ҵ�������Ԫ�ص�λ�ã���Ҫ����λ�ú����Ԫ�غ��� 
     for (i=len-1; i>high; i--)  // Ԫ�غ��� 
     {   
           src[i+1] = src[i];             
     }
     //src[i+1] = src[i];
     if(flag)//�߽��ǰ��
         src[i] = key;
     else //�ͽ�����
         src[i+1] = key; // ����
     return high;
        
}
//Ԫ�ز����ð�ݷ�����Ȼ�󷵻�Ԫ�ص����кţ�ʧ�ܷ���-1��
int BubbleInsert(unsigned short* src, unsigned char len,unsigned short key)
{
    unsigned char  i,j, unchange;
    unsigned short tmp;
     if(src[len-1]!=0)//������һ��Ԫ�ز��գ�˵����������
         return -1;
     if(key==0) 
         return -1;
     //��ȷ������Ԫ�ر�
     while((len>(unsigned char)1)&&(src[len-1]==0)) 
         len--;
     
     if(src[len-1] == 0)//�����λ��Ԫ��Ϊ0��˵����ʱ��Ϊ1
     {
         src[len-1] = key;
         return len-1;         
     }
     else
         len++;
     
     for(i=0;i<len-1;i++)
     {
         if(key == src[i])
            return (char)i;               
     }    
     src[i]=key;
         
     for (i=0; i<len; i++)
     {
        unchange = (unsigned char)1;
        for (j=0; j<len-1-i; j++)
        {
           if (src[j] > src[j+1])
           {
              unchange = 0;
              tmp = src[j];
              src[j] = src[j+1];
              src[j+1] = tmp;
           }
        }
        if (unchange == (unsigned char)1) //�����б仯����������
        break;
     }
     for(i=0;i<len;i++)
     {
         if(key == src[i])
            return (char)i;               
     } 
     return (char)(-1);
}

//���ַ���ȡ����Ԫ�أ�ʧ�ܷ���-1
int GetBinSearchValue(unsigned short *src, unsigned char len, unsigned char index)
{       
   unsigned char low,high, mid;//�õ�ǰ���������ϡ��½�ĳ�ֵ
   
   if(index > len-1) return -1;
 
   while((len>1)&&(src[len-1]==0)) len--;
   low = 0;
   high = len-1;
   if((src[index] < src[low])||(src[index] > src[high]))//Ԫ�ز��ڴα���
       return -1;
   if(src[index] == src[low])
       return src[low];
   if(src[index] == src[high])
       return src[high];
   while(low < high)//��ǰ��������R[low..high]�ǿ�
   {
      mid = low + ((high-low)/2); //ʹ�� (low + high) / 2 �����������������
   
      if(src[index] == src[mid]) 
          return src[mid]; //���ҳɹ�����
   
      if(src[index] < src[mid])
      {
         high = mid-1; //������R[low..mid-1]�в���
         if(src[index] == src[high])//����ϱ߽��Ԫ������Ҫ�ҵ�Ԫ��
             return src[high];
      }
      else
      {
         low = mid+1; //������R[mid+1..high]�в���
         if(src[index] == src[low])//����±߽��Ԫ������Ҫ�ҵ�Ԫ��
             return src[low];
      }
    }
    return -1; //��low>highʱ��ʾ��������Ϊ�գ�����ʧ��
} //BinSeareh

//��ȡ�������������ֵ
int GetBubbleMaxValue(unsigned short *src, unsigned char len )
{       

   while((len>1)&&(src[len-1]==0)) len--;
   
    return src[len-1]; 
} 

//��ȡ�������������ֵ������
int GetBubbleMaxIndex(unsigned short *src, unsigned char len )
{       

   while((len>1)&&(src[len-1]==0)) len--;
   
    return len-1; 
}



//----------------------------------------------------------------------------
//�������ƣ�rms(Root Mean Square)
//��ڲ�����unsigned int* pdata	ָ��Դ���ݱ��ָ��
//			unsigned char N	Ԫ�صĸ���
//���ز���: 0		����Ԫ�ؾ��Ϸ�
//	    0xff	û�кϷ�Ԫ��
//	    float       result    
//��������: �ú���ʵ�ֶ�Ԫ���������ֵ	
//----------------------------------------------------------------------------

float rms(unsigned int* pdata,unsigned char N,unsigned char* err)
{
	unsigned char i = 0;
	double sum = 0;//32λ������
	float result = 0;

	if(N == 0)
	{
		*err = 0xff;
		return 0; 
	}
	else
	{
		for(i = 0;i < N;i++)
		{
			//�Ȱ�ָ�����ת����double�ͣ���ȡ������
			//sum += (long double)pow(*((double*)pdata),2);
			sum += (double)(((*(unsigned int*)pdata)*(*(unsigned int*)pdata))/N);
			pdata++;
		}
		
		result = (float)sqrt(sum);//�ٿ���
		
		*err = 0;	
		return result;	
			
	}	
	
}	

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

float average(unsigned int* psrc,unsigned char N,unsigned char* err)
{
	//INT8U errMem_Get,errMem_Put;
	//INT16U *pdata;//����һ��Դ���ݵĿ���
	unsigned int Mem_Comm[RCV_CAP_BUF_SIZE];
	
	unsigned char n = N;		 //����Ԫ�ص�ԭʼ����
	unsigned char i = 0;
	//unsigned int* pdata_dst= (unsigned int*)0;
	unsigned int elem_max = 0;
	unsigned int elem_min = 0;
	unsigned char pos_max = 0;
	unsigned char pos_min = 0;
	float sum_avg = 0;	//32λ������
	float sum_sqr = 0;	
	unsigned int* pdata_curr;//�Ƿ�Ԫ�صĵ�ǰλ��	
	unsigned int* pdata_next;//�Ƿ�Ԫ�ص���һ��λ��	
	float toler_std = 0;
	
	unsigned int *pdata = Mem_Comm;
	unsigned int *pdst = Mem_Comm;

	//����һ���ڴ�
	/*if (pMem_Comm != (OS_MEM*)0) //���ڴ洴���ɹ�
	{		
		pdata_dst = OSMemGet(pMem_Comm, &errMem_Get);
		if(errMem_Get == OS_NO_ERR)
		{
			//�ڴ����ɹ�
			pdata = pdata_dst;
		}
		else if(errMem_Get == OS_MEM_NO_FREE_BLKS)
		{
			//û���ڴ����
			*err = OS_MEM_NO_FREE_BLKS;
			return 0;//	
		}	
	}*/
	//�ڴ����ɹ���
	//��Դ���ݵ����ݿ������������ڴ�ռ�
	
	
	//pdata = pdata_dst;	
	for(i = 0;i < n;i++)
	{
		*pdata = *psrc; 
		pdata++;
		psrc++;	
	}
	//���б���Ԫ�صľ�ƽ�������Ԫ����������λ�ò����Ĵ�С	
	while(1)
	{
		if(n == 0)
		{
			*err = 0xff;//û�кϷ�����
			sum_avg = 0;
			break;//goto Mem_Free;
		}			
		else
		{
			//������ƽ��ֵ
			pdata = pdst;//ָ���ñ���
			sum_avg = 0;//ƽ��ֵ��ʼ��Ϊ0	
			for(i = 0;i < n;i++)
			{
				sum_avg += (float)((float)(*pdata)/(float)n);
				pdata++;	
			}
			//�������(��׼��)
			pdata = pdst;//ָ���ñ���
			sum_sqr = 0;//ƽ��ֵ��ʼ��Ϊ0
			for(i = 0;i < n;i++)
			{
				sum_sqr += ((float)(*pdata) - (sum_avg)) * ((float)(*pdata) - (sum_avg));
				pdata++;
			}
			toler_std = sqrt(sum_sqr/(n-1));
			
			//��������꣬������ѭ��
			if(((float)toler_std/(float)sum_avg) <= 0.1)
			{	
				*err = 0;			
				break;//goto Mem_Free;
			}
				
			pdata = pdst;//ָ���ñ���	
			//����Ͳ��Ҳ�ȥ�������ֵ����Сֵ
			elem_max = pdata[0];//�����ֵ����Сֵָ���һ��Ԫ��
			elem_min = pdata[0];
			for(i = 0;i < n; i++)
			{
				if(elem_max <= pdata[i])
				{
					elem_max = pdata[i];
					pos_max = i;//��¼���Ԫ��λ��
				}
				else if(elem_min >= pdata[i])
				{
					elem_min = pdata[i];
					pos_min = i;//��¼��СԪ��λ��
				}
				pdata++;
			}
			pdata = pdst;//ָ���ñ���
			//ȥ�����ֵ
			pdata_curr = &pdata[pos_max];	//���浱ǰλ��	
			pdata_next = &pdata[pos_max+1];	//ָ��ǰλ�õ���һ��Ԫ��λ��
			for(i = 0;i < n-pos_max-1;i++)//ɾ���ҵ���Ԫ�أ��Ѻ��Ԫ��ǰ��
			{				
				*pdata_curr = *pdata_next;//��һ��Ԫ��ǰ��
				pdata_curr++;
				pdata_next++;
			}
			n--;
			
			//ȥ����Сֵ
			pdata_curr = &pdata[pos_min];	//���浱ǰλ��	
			pdata_next = &pdata[pos_min+1];	//ָ��ǰλ�õ���һ��Ԫ��λ��
			for(i = 0;i < n-pos_min-1;i++)//ɾ���ҵ���Ԫ�أ��Ѻ��Ԫ��ǰ��
			{				
				*pdata_curr = *pdata_next;//��һ��Ԫ��ǰ��
				pdata_curr++;
				pdata_next++;
			}
			n--;
							 
			if((float)((float)(N - n)/(float)N) > 0.5)//�Ƿ�Ԫ����ռ�ı���
			{
				*err = 0xff;
				sum_avg = 0;
				break;//goto Mem_Free;
			}
		}					
	}
//Mem_Free:
	//�ͷ��ڴ�ռ�
	//errMem_Put = OSMemPut(pMem_Comm, (void*)pdata_dst);
	//if(errMem_Put == OS_NO_ERR) 
	//{
		//�ڴ��ͷųɹ�		
	//}
	//else if(errMem_Put == OS_MEM_FULL)
	//{
		//�ڴ����Ѿ������ٽ��ܸ����ͷŵ��ڴ�顣�������˵���û���������˴�
		//���ͷ��˶�����OSMemGet���������õ����ڴ�顣
		//���𣬵ȴ��ͷ��ڴ�
	//	while(1);
	//}		
	return (float)(sum_avg);//����ƽ��ֵ
}

/* ��Ȩ����ƽ���˲��� */ 
/*#define N 12
uint8_t code_coe[N] = {1,2,3,4,5,6,7,8,9,10,11,12};
uint8_t sum_coe = 1+2+3+4+5+6+7+8+9+10+11+12;
uint16_t filter()
{
    uint8_t count;
    uint16_t value_buf[N];
    uint32_t sum = 0;
    for(count = 0; count < N; count++)
    {
        value_buf[count] = ;
        
    }
    for(count =0; count < N; count++ )
    {
        sum += value_buf[count] * code_coe[count];
        
    }
    return (char)(sum/sum_coe);
}*/
