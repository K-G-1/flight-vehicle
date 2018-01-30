#ifndef __hight_H
#define __hight_H

struct _Hight
{
	int origin ;
	int histor ;
	int averag;
};

extern struct _Hight Hight ;
extern unsigned int g_Set_Hight;
extern float g_Alt_Hight;
void hight_init(void );
void fifter_hight(unsigned int set_hight);


#endif


