#ifndef __DDP_COLOR_INDEX_H__
#define __DDP_COLOR_INDEX_H__

#include "ddp_drv.h"


static DISPLAY_PQ_T pqindex =
{
GLOBAL_SAT   :
{0x80,0x82,0x84,0x86,0x88,0x8a,0x8c,0x8e,0x90,0x92,0x94,0x96,0x98,0x9a,0x9c,0x9e,0xa0,0xa2,0xa4,0xa6}, //0~9

PURP_TONE_S  :
{//hue 0~2
	{//0 disable  
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80}
	}, 

	{//1
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80}
	},
	
	{//2
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80}
	},
	{//3
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80}
	},
	
	{//4
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80}
	},
	
	{//5
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80}
	},

	{//6
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80}
	},
  // 7
	{
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80}
	},
	// 8
	{
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80}
	},
	// 9
	{
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80}
	},
	// 10
	{
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80}
	},
	// 11
	{
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80}
	},
	// 12
	{
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80}
	},
	// 13
	{
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80}
	},
	// 14
	{
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80}
	},
	// 15
	{
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80}
	},
	// 16
	{
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80}
	},
	// 17
	{
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80}
	},
	// 18
	{
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80}
	},
	// 19
	{
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80}
	}
},
SKIN_TONE_S:
{// hue 3 ~ 10
	{//0 disable  
		{0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80}
	},

	{// 1
		{0x90, 0x90, 0x90, 0x90, 0x90,0x90,0x90,0x80},          
		{0x8a, 0x8a, 0x8a, 0x8a, 0x8a,0x8a,0x8a,0x80},          
		{0x80, 0x80, 0x80, 0x80, 0x80,0x80,0x80,0x80},          
		{0x1a, 0x1a, 0x18, 0x17, 0x1c,0x1e,0x1e,0x1c},          
		{0x34, 0x36, 0x30, 0x2f, 0x38,0x3c,0x3c,0x38}           
	}, 

	{// 2
		{0x90, 0x98, 0x98, 0x98, 0x98,0x98,0x98,0x80},          
		{0x8c, 0x8c, 0x8c, 0x8c, 0x8c,0x8c,0x8c,0x80},          
		{0x80, 0x80, 0x80, 0x80, 0x80,0x80,0x80,0x80},          
		{0x1a, 0x1a, 0x18, 0x17, 0x1c,0x1e,0x1e,0x1c},          
		{0x34, 0x36, 0x30, 0x2f, 0x38,0x3c,0x3c,0x38}           
	},                                              

	{// 3
		{0x90, 0xa1, 0xa1, 0xa1, 0xa1,0xa1,0xa1,0x80},          
		{0x8f, 0x8f, 0x8f, 0x8f, 0x8f,0x8f,0x8f,0x80},          
		{0x80, 0x80, 0x80, 0x80, 0x80,0x80,0x80,0x80},          
		{0x1a, 0x1a, 0x18, 0x17, 0x1c,0x1e,0x1e,0x1c},          
		{0x34, 0x36, 0x30, 0x2f, 0x38,0x3c,0x3c,0x38}           
	},                                                  

	{// 4
		{0x95, 0xa9, 0xa9, 0xa9, 0xa9,0xa9,0xa9,0x80},          
		{0x8f, 0x97, 0x97, 0x97, 0x97,0x97,0x97,0x80},          
		{0x7a, 0x7a, 0x7a, 0x7a, 0x7a,0x7a,0x7a,0x80},          
		{0x1a, 0x1a, 0x18, 0x17, 0x1c,0x1e,0x1e,0x1c},          
		{0x34, 0x36, 0x30, 0x2f, 0x38,0x3c,0x3c,0x38}           
	},                                                    
	                                                          
	{// 5
		{0x9a, 0xb2, 0xb2, 0xb2, 0xb2,0xb2,0xb2,0x80},          
		{0x8f, 0xa0, 0xa0, 0xa0, 0xa0,0xa0,0xa0,0x80},          
		{0x75, 0x75, 0x75, 0x75, 0x75,0x75,0x75,0x80},          
		{0x16, 0x18, 0x14, 0x12, 0x1a,0x1e,0x1e,0x1a},          
		{0x2d, 0x30, 0x27, 0x24, 0x34,0x3c,0x3c,0x34}           
	},                                                     
	                                                          
	{// 6
		{0x9a, 0xb2, 0xba, 0xba, 0xba,0xba,0xb2,0x80},          
		{0x93, 0xa5, 0xa5, 0xa5, 0xa5,0xa5,0xa5,0x80},          
		{0x6f, 0x6f, 0x6f, 0x6f, 0x6f,0x6f,0x6f,0x80},          
		{0x16, 0x18, 0x14, 0x12, 0x1a,0x1e,0x1e,0x1a},          
		{0x2d, 0x30, 0x27, 0x24, 0x34,0x3c,0x3c,0x34}           
	},                                                       
                                                            
	{// 7
		{0x9a, 0xb2, 0xc3, 0xc3, 0xc3,0xc3,0xb2,0x80},          
		{0x97, 0xab, 0xab, 0xab, 0xab,0xab,0xab,0x80},          
		{0x6a, 0x6a, 0x6a, 0x6a, 0x6a,0x6a,0x6a,0x80},          
		{0x16, 0x18, 0x14, 0x12, 0x1a,0x1e,0x1e,0x1a},          
		{0x2d, 0x30, 0x27, 0x24, 0x34,0x3c,0x3c,0x34}           
	},                                                        
                                                            
	{//8
		{0xa0, 0xba, 0xcb, 0xcb, 0xcb,0xcb,0xb6,0x80},          
		{0x9f, 0xb0, 0xb0, 0xb0, 0xb0,0xb0,0xaa,0x80},          
		{0x64, 0x64, 0x64, 0x64, 0x64,0x64,0x64,0x80},          
		{0x16, 0x18, 0x14, 0x12, 0x1a,0x1e,0x1e,0x1a},          
		{0x2d, 0x30, 0x27, 0x24, 0x34,0x3c,0x3c,0x34}           
	},                                                       
	                                                          
	{// 9
		{0xa7, 0xc3, 0xd3, 0xd3, 0xd3,0xd3,0xbb,0x80},          
		{0xa7, 0xb6, 0xb6, 0xb6, 0xb6,0xb6,0xa9,0x80},          
		{0x5f, 0x5f, 0x5f, 0x5f, 0x5f,0x5f,0x5f,0x80},          
		{0x16, 0x18, 0x14, 0x12, 0x1a,0x1e,0x1e,0x1a},          
		{0x2d, 0x30, 0x27, 0x24, 0x34,0x3c,0x3c,0x34}           
	},                                                      
	                                                          
	{// 10
		{0xa7, 0xc3, 0xdd, 0xdd, 0xdd,0xdd,0xbb,0x80},          
		{0xa7, 0xb6, 0xbb, 0xbb, 0xbb,0xbb,0xa9,0x80},          
		{0x5f, 0x5a, 0x5a, 0x5a, 0x5a,0x5a,0x5a,0x80},          
		{0x16, 0x18, 0x14, 0x12, 0x1a,0x1e,0x1e,0x1a},          
		{0x2d, 0x30, 0x27, 0x24, 0x34,0x3c,0x3c,0x34}           
	},
	
	{// 11
		{0xa7, 0xc3, 0xe7, 0xe7, 0xe7,0xe7,0xbb,0x80},          
		{0xa7, 0xb6, 0xc0, 0xc0, 0xc0,0xc0,0xa9,0x80},          
		{0x5f, 0x55, 0x55, 0x55, 0x55,0x55,0x55,0x80},          
		{0x16, 0x18, 0x14, 0x12, 0x1a,0x1e,0x1e,0x1a},          
		{0x2d, 0x30, 0x27, 0x24, 0x34,0x3c,0x3c,0x34}           
	},

    {// 12
		{0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80}         
	},                                                        
	
	{// 13
		{0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80}
	},
	
	{// 14
		{0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80}
	},
	
	{// 15
		{0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80}
	},
	
	{// 16
		{0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80}
	},
	
	{// 17
		{0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80}
	},
	
	{// 18
		{0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80}
	},
	
	{// 19
		{0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80}
	}
},
GRASS_TONE_S:
{// hue 11 ~ 16
    {//0 disable  
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80}
    },

    {// 1
        {0x90,0x90, 0x90, 0x90, 0x90, 0x80},    
        {0x8a,0x8a, 0x8a, 0x8a, 0x8a, 0x80},    
        {0x80,0x80, 0x80, 0x80, 0x80, 0x80},    
        {0x18,0x1a, 0x1e, 0x18, 0x1a, 0x1a},    
        {0x31,0x35, 0x3c, 0x30, 0x34, 0x36}     
    },   

    {// 2
        {0x98,0x98, 0x98, 0x98, 0x98, 0x80},    
        {0x8c,0x8c, 0x8c, 0x8c, 0x8c, 0x80},    
        {0x80,0x80, 0x80, 0x80, 0x80, 0x80},    
        {0x18,0x1a, 0x1e, 0x18, 0x1a, 0x1a},    
        {0x31,0x35, 0x3c, 0x30, 0x34, 0x36}     
    },                                      
                                      
    {// 3                                      
        {0xa1,0xa1, 0xa0, 0xa0, 0xa0, 0x80},    
        {0x8f,0x8f, 0x8f, 0x8f, 0x8f, 0x80},    
        {0x80,0x80, 0x80, 0x80, 0x80, 0x80},    
        {0x18,0x1a, 0x1e, 0x18, 0x1a, 0x1a},    
        {0x31,0x35, 0x3c, 0x30, 0x34, 0x36}     
    },                                      
                                      
    {// 4                                      
        {0xa5,0xa5, 0xa5, 0xa5, 0xa0, 0x80},    
        {0x97,0x97, 0x97, 0x97, 0x8f, 0x80},    
        {0x7a,0x7a, 0x7a, 0x7a, 0x7a, 0x80},    
        {0x18,0x1a, 0x1e, 0x18, 0x1a, 0x1a},    
        {0x31,0x35, 0x3c, 0x30, 0x34, 0x36}     
    },                                       
                                    
    {// 5
        {0xa9,0xa9, 0xa9, 0xa9, 0xa0, 0x80},    
        {0xa0,0xa0, 0xa0, 0xa0, 0x8f, 0x80},    
        {0x75,0x75, 0x75, 0x75, 0x75, 0x80},    
        {0x14,0x17, 0x1e, 0x13, 0x16, 0x18},    
        {0x28,0x2f, 0x3c, 0x26, 0x2d, 0x30}     
    },                                      
                                    
    {// 6
        {0xba,0xba, 0xba, 0xba, 0xa0, 0x80},    
        {0xa5,0xa5, 0xa5, 0xa5, 0x93, 0x80},    
        {0x6f,0x6f, 0x6f, 0x6f, 0x6f, 0x80},    
        {0x14,0x17, 0x1e, 0x13, 0x16, 0x18},    
        {0x28,0x2f, 0x3c, 0x26, 0x2d, 0x30}     
    },                                        
                                      
    {// 7                                      
        {0xc3,0xc3, 0xc3, 0xc3, 0xa0, 0x80},    
        {0xab,0xab, 0xab, 0xab, 0x98, 0x80},    
        {0x6a,0x6a, 0x6a, 0x6a, 0x6a, 0x80},    
        {0x14,0x17, 0x1e, 0x13, 0x16, 0x18},    
        {0x28,0x2f, 0x3c, 0x26, 0x2d, 0x30}     
    },                                       
                                      
    {// 8                                      
        {0xc4,0xcb, 0xcb, 0xc6, 0xa8, 0x80},    
        {0xb0,0xb0, 0xb0, 0xb0, 0x9e, 0x80},    
        {0x64,0x64, 0x64, 0x64, 0x6a, 0x80},    
        {0x14,0x17, 0x1e, 0x13, 0x16, 0x18},    
        {0x28,0x2f, 0x3c, 0x26, 0x2d, 0x30}     
    },                                        
                                    
    {// 9                                      
        {0xc6,0xd3, 0xd3, 0xca, 0xb0, 0x80},    
        {0xb6,0xb6, 0xb6, 0xb6, 0xa5, 0x80},    
        {0x5f,0x5f, 0x5f, 0x5f, 0x6a, 0x80},    
        {0x14,0x17, 0x1e, 0x13, 0x16, 0x18},    
        {0x28,0x2f, 0x3c, 0x26, 0x2d, 0x30}     
    },                                       
                                    
    {// 10                                      
        {0xc6,0xdd, 0xdd, 0xd2, 0xb6, 0x80},    
        {0xb4,0xbb, 0xbb, 0xb6, 0xa8, 0x80},    
        {0x5c,0x5a, 0x5a, 0x5f, 0x6a, 0x80},    
        {0x14,0x17, 0x1e, 0x13, 0x16, 0x18},    
        {0x28,0x2f, 0x3c, 0x26, 0x2d, 0x30}     
    },                                        
                                      
    {//  11                                     
        {0xc6,0xe7, 0xe7, 0xdb, 0xbd, 0x80},    
        {0xb2,0xc0, 0xc0, 0xb6, 0xab, 0x80},    
        {0x5a,0x55, 0x55, 0x5f, 0x6a, 0x80},    
        {0x14,0x17, 0x1e, 0x13, 0x16, 0x18},    
        {0x28,0x2f, 0x3c, 0x26, 0x2d, 0x30}     
    },                                         
                                      
    {//  12                                     
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80}    
    }, 

    {// 13
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80}
    },

    {// 14
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80}
    },

    {// 15
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80}
    },

    {// 16
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80}
    },

    {// 17
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80}
    },

    {// 18
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80}
    },

    {// 19
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80}
    }
},
SKY_TONE_S:
{// hue 17~ 19
	{//0 disable
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80}
	},
		{// 1
		{0x90, 0x90, 0x90},
		{0x8a, 0x8a, 0x8a},
		{0x80, 0x80, 0x80},
		{0x17, 0x1e, 0x1c},
		{0x2f, 0x3c, 0x38}
	},
		{// 2
		{0x98, 0x98, 0x98},
		{0x8c, 0x8c, 0x8c},
		{0x80, 0x80, 0x80},
		{0x17, 0x1e, 0x1c},
		{0x2f, 0x3c, 0x38}
	},
	{// 3
		{0xa1, 0xa1, 0xa1},
		{0x8f, 0x8f, 0x8f},
		{0x80, 0x80, 0x80},
		{0x17, 0x1e, 0x1c},
		{0x2f, 0x3c, 0x38}
	},
	{// 4
		{0xa6, 0xa9, 0xa6},
		{0x94, 0x97, 0x94},
		{0x7a, 0x7a, 0x7a},
		{0x17, 0x1e, 0x1c},
		{0x2f, 0x3c, 0x38}
	},

	{// 5
		{0xab, 0xb2, 0xab},
		{0x9a, 0xa0, 0x9a},
		{0x75, 0x75, 0x75},
		{0x12, 0x1e, 0x1a},
		{0x24, 0x3c, 0x34}
	},

	{// 6
		{0xab, 0xba, 0xab},
		{0x9e, 0xa5, 0x9e},
		{0x6f, 0x6f, 0x6f},
		{0x12, 0x1e, 0x1a},
		{0x24, 0x3c, 0x34}
	},
	{// 7
		{0xad, 0xc3, 0xad},
		{0xa3, 0xab, 0xa3},
		{0x6a, 0x6a, 0x6a},
		{0x12, 0x1e, 0x1a},
		{0x24, 0x3c, 0x34}
	},
	{//8
		{0xb5, 0xcb, 0xb5},
		{0xa8, 0xb0, 0xa8},
		{0x68, 0x64, 0x68},
		{0x12, 0x1e, 0x1a},
		{0x24, 0x3c, 0x34}
	},

	{//9
		{0xbd, 0xd3, 0xbd},
		{0xad, 0xb6, 0xad},
		{0x67, 0x5f, 0x67},
		{0x12, 0x1e, 0x1a},
		{0x24, 0x3c, 0x34}
	},

	{//10
		{0xc3, 0xdd, 0xc3},
		{0xaf, 0xbb, 0xaf},
		{0x63, 0x5a, 0x63},
		{0x12, 0x1e, 0x1a},
		{0x24, 0x3c, 0x34}
	},
	
	{//11
		{0xca, 0xe7, 0xca},
		{0xb2, 0xc0, 0xb4},
		{0x5f, 0x55, 0x5f},
		{0x12, 0x1e, 0x1a},
		{0x24, 0x3c, 0x34}
	},
	
	{//12
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80}
	},
	
	{//13
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80}
	},
	
	{//14
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80}
	},
	
	{//15
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80}
	},
	
	{//16
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80}
	},
	
	{//17
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80}
	},
	
	{//18
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80}
	},
	
	{//19
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80},
		{0x80, 0x80, 0x80}
	}
},

PURP_TONE_H :
{ 
//hue 0~2 
    {0x80, 0x80, 0x80},//3 
    {0x80, 0x80, 0x80},//4 
    {0x80, 0x80, 0x80},
    {0x80, 0x80, 0x80},//3 
    {0x80, 0x80, 0x80},//4 
    {0x80, 0x80, 0x80},
    {0x80, 0x80, 0x80},
    {0x80, 0x80, 0x80},
    {0x80, 0x80, 0x80},
    {0x80, 0x80, 0x80},
    {0x80, 0x80, 0x80},//3 
    {0x80, 0x80, 0x80},//4 
    {0x80, 0x80, 0x80},
    {0x80, 0x80, 0x80},//3 
    {0x80, 0x80, 0x80},//4 
    {0x80, 0x80, 0x80},
    {0x80, 0x80, 0x80},
    {0x80, 0x80, 0x80},
    {0x80, 0x80, 0x80},
    {0x80, 0x80, 0x80}
},
	
SKIN_TONE_H:
{
//hue 3~10
    {0x80, 0x80,0x71, 0x67,  0x56,  0x5d,  0x67,  0x80},// 0 (-9)
    {0x80, 0x80,0x72, 0x68,  0x5a,  0x61,  0x6a,  0x80},// 1 (-8)
    {0x80, 0x80,0x73, 0x69,  0x5e,  0x64,  0x6e,  0x80},// 2 (-7)
    {0x80, 0x80,0x74, 0x6a,  0x63,  0x67,  0x71,  0x80},// 3 (-6)
    {0x80, 0x80,0x75, 0x6b,  0x68,  0x6a,  0x74,  0x80},// 4 (-5)
    {0x80, 0x80,0x77, 0x6d,  0x6d,  0x6d,  0x78,  0x80},// 5 (-4)     
    {0x80, 0x80,0x77, 0x72,  0x72,  0x72,  0x78,  0x80},// 6 (-3)     
    {0x80, 0x80,0x77, 0x77,  0x77,  0x77,  0x78,  0x80},// 7 (-2)
    {0x80, 0x80,0x7b, 0x7b,  0x7b,  0x7b,  0x7c,  0x80},// 8 (-1)
    {0x80, 0x80,0x80, 0x80,  0x80,  0x80,  0x80,  0x80},// disable    
    {0x82, 0x85,0x85, 0x85,  0x85,  0x85,  0x85,  0x80},// 10 (+1)
    {0x84, 0x8a,0x8a, 0x8a,  0x8a,  0x8a,  0x8a,  0x80},// 11 (+2)
    {0x85, 0x8a,0x8a, 0x8f,  0x8f,  0x8f,  0x8d,  0x80},// 12 (+3)
    {0x86, 0x8b,0x93, 0x94,  0x94,  0x94,  0x90,  0x80},// 13 (+4)
    {0x88, 0x8d,0x97, 0x98,  0x98,  0x96,  0x90,  0x80},// 14 (+5)
    {0x8a, 0x90,0x9c, 0x9c,  0x9c,  0x99,  0x90,  0x80},// 15 (+6)
    {0x8c, 0x92,0xa0, 0xa0,  0xa0,  0x9c,  0x90,  0x80},// 16 (+7)
    {0x8e, 0x94,0xa4, 0xa4,  0xa4,  0x9f,  0x90,  0x80},// 17 (+8)
    {0x90, 0x96,0xa8, 0xa8,  0xa8,  0xa2,  0x90,  0x80},// 18 (+9)
    {0x80, 0x80,0x80, 0x80,  0x80,  0x80,  0x80,  0x80}// 19 
},

	
GRASS_TONE_H :
{
// hue 11~16
    {0x71, 0x63, 0x68, 0x62, 0x69, 0x80},// 0 (-9)
    {0x72, 0x65, 0x5c, 0x64, 0x6b, 0x80},// 1 (-8)
    {0x73, 0x67, 0x60, 0x66, 0x6d, 0x80},// 2 (-7)
    {0x74, 0x69, 0x64, 0x68, 0x6f, 0x80},// 3 (-6)
    {0x75, 0x6b, 0x68, 0x6a, 0x71, 0x80},// 4 (-5)
    {0x77, 0x6e, 0x6d, 0x6d, 0x74, 0x80},// 5 (-4)     
    {0x77, 0x72, 0x72, 0x72, 0x77, 0x80},// 6 (-3)     
    {0x77, 0x77, 0x77, 0x77, 0x7a, 0x80},// 7 (-2)
    {0x78, 0x78, 0x78, 0x78, 0x7d, 0x80},// 8 (-1)
    {0x80, 0x80, 0x80, 0x80, 0x80, 0x80},// disable    
    {0x85, 0x85, 0x85, 0x85, 0x83, 0x00},// 10 (+1)
    {0x8a, 0x8a, 0x8a, 0x8a, 0x86, 0x80},// 11 (+2)
    {0x8c, 0x8f, 0x8f, 0x8d, 0x88, 0x00},// 12 (+3)
    {0x8e, 0x94, 0x94, 0x91, 0x8a, 0x80},// 13 (+4)
    {0x90, 0x98, 0x98, 0x94, 0x8b, 0x00},// 14 (+5)
    {0x92, 0x9c, 0x9c, 0x98, 0x8d, 0x80},// 15 (+6)
    {0x94, 0xa1, 0xa1, 0x9c, 0x8f, 0x80},// 16 (+7)
    {0x96, 0xa5, 0xa5, 0xa0, 0x91, 0x80},// 17 (+8)
    {0x98, 0xa9, 0xa9, 0xa4, 0x93, 0x80},// 18 (+9)
    {0x80, 0x80, 0x80, 0x80,  0x80,  0x80}// 19     
},

SKY_TONE_H:
{//17 ~ 19
    {0x6b, 0x5e, 0x6b},// 0 (-9)
    {0x6d, 0x60, 0x6d},// 1 (-8)
    {0x6f, 0x61, 0x6f},// 2 (-7)
    {0x70, 0x63, 0x70},// 3 (-6)
    {0x71, 0x68, 0x71},// 4 (-5)
    {0x72, 0x6d, 0x72},// 5 (-4)     
    {0x74, 0x72, 0x74},// 6 (-3)     
    {0x77, 0x77, 0x77},// 7 (-2)
    {0x7b, 0x7b, 0x7b},// 8 (-1)
    {0x80, 0x80, 0x80},// disable    
    {0x85, 0x85, 0x85},// 10 (+1)
    {0x8a, 0x8a, 0x8a},// 11 (+2)
    {0x8c, 0x8f, 0x8c},// 12 (+3)
    {0x8f, 0x94, 0x8f},// 13 (+4)
    {0x92, 0x98, 0x92},// 14 (+5)
    {0x95, 0x9c, 0x95},// 15 (+6)
    {0x96, 0x9e, 0x99},// 16 (+7)
    {0x97, 0x9f, 0x97},// 17 (+8)
    {0x99, 0xa1, 0x99},// 18 (+9)    
    {0x80, 0x80, 0x80}// 19  
}

};

const DISPLAY_GAMMA_T gammaindex = 
{
entry:
{
    {
            0,   4,   8,  12,  16,  20,  24,  28,  32,  36,  40,  44,  48,  52,  56,  60,  64,  68,  72,  76,  80,  84,  88,  92,  96,
        100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144, 148, 152, 156, 160, 164, 168, 172, 176, 180, 184, 188, 192, 196,
        200, 204, 208, 212, 216, 220, 224, 228, 232, 236, 240, 244, 248, 252, 256, 260, 264, 268, 272, 276, 280, 284, 288, 292, 296,
        300, 304, 308, 312, 316, 320, 324, 328, 332, 336, 340, 344, 348, 352, 356, 360, 364, 368, 372, 376, 380, 384, 388, 392, 396,
        400, 404, 408, 412, 416, 420, 424, 428, 432, 436, 440, 444, 448, 452, 456, 460, 464, 468, 472, 476, 480, 484, 488, 492, 496,
        500, 504, 508, 512, 516, 520, 524, 528, 532, 536, 540, 544, 548, 552, 556, 560, 564, 568, 572, 576, 580, 584, 588, 592, 596,
        600, 604, 608, 612, 616, 620, 624, 628, 632, 636, 640, 644, 648, 652, 656, 660, 664, 668, 672, 676, 680, 684, 688, 692, 696,
        700, 704, 708, 712, 716, 720, 724, 728, 732, 736, 740, 744, 748, 752, 756, 760, 764, 768, 772, 776, 780, 784, 788, 792, 796,
        800, 804, 808, 812, 816, 820, 824, 828, 832, 836, 840, 844, 848, 852, 856, 860, 864, 868, 872, 876, 880, 884, 888, 892, 896,
        900, 904, 908, 912, 916, 920, 924, 928, 932, 936, 940, 944, 948, 952, 956, 960, 964, 968, 972, 976, 980, 984, 988, 992, 996,
        1000, 1004, 1008, 1012, 1016, 1020, 1023
    },
    {
            0,   4,   8,  12,  16,  20,  24,  28,  32,  36,  40,  44,  48,  52,  56,  60,  64,  68,  72,  76,  80,  84,  88,  92,  96,
        100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144, 148, 152, 156, 160, 164, 168, 172, 176, 180, 184, 188, 192, 196,
        200, 204, 208, 212, 216, 220, 224, 228, 232, 236, 240, 244, 248, 252, 256, 260, 264, 268, 272, 276, 280, 284, 288, 292, 296,
        300, 304, 308, 312, 316, 320, 324, 328, 332, 336, 340, 344, 348, 352, 356, 360, 364, 368, 372, 376, 380, 384, 388, 392, 396,
        400, 404, 408, 412, 416, 420, 424, 428, 432, 436, 440, 444, 448, 452, 456, 460, 464, 468, 472, 476, 480, 484, 488, 492, 496,
        500, 504, 508, 512, 516, 520, 524, 528, 532, 536, 540, 544, 548, 552, 556, 560, 564, 568, 572, 576, 580, 584, 588, 592, 596,
        600, 604, 608, 612, 616, 620, 624, 628, 632, 636, 640, 644, 648, 652, 656, 660, 664, 668, 672, 676, 680, 684, 688, 692, 696,
        700, 704, 708, 712, 716, 720, 724, 728, 732, 736, 740, 744, 748, 752, 756, 760, 764, 768, 772, 776, 780, 784, 788, 792, 796,
        800, 804, 808, 812, 816, 820, 824, 828, 832, 836, 840, 844, 848, 852, 856, 860, 864, 868, 872, 876, 880, 884, 888, 892, 896,
        900, 904, 908, 912, 916, 920, 924, 928, 932, 936, 940, 944, 948, 952, 956, 960, 964, 968, 972, 976, 980, 984, 988, 992, 996,
        1000, 1004, 1008, 1012, 1016, 1020, 1023
    },
    {
            0,   4,   8,  12,  16,  20,  24,  28,  32,  36,  40,  44,  48,  52,  56,  60,  64,  68,  72,  76,  80,  84,  88,  92,  96,
        100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144, 148, 152, 156, 160, 164, 168, 172, 176, 180, 184, 188, 192, 196,
        200, 204, 208, 212, 216, 220, 224, 228, 232, 236, 240, 244, 248, 252, 256, 260, 264, 268, 272, 276, 280, 284, 288, 292, 296,
        300, 304, 308, 312, 316, 320, 324, 328, 332, 336, 340, 344, 348, 352, 356, 360, 364, 368, 372, 376, 380, 384, 388, 392, 396,
        400, 404, 408, 412, 416, 420, 424, 428, 432, 436, 440, 444, 448, 452, 456, 460, 464, 468, 472, 476, 480, 484, 488, 492, 496,
        500, 504, 508, 512, 516, 520, 524, 528, 532, 536, 540, 544, 548, 552, 556, 560, 564, 568, 572, 576, 580, 584, 588, 592, 596,
        600, 604, 608, 612, 616, 620, 624, 628, 632, 636, 640, 644, 648, 652, 656, 660, 664, 668, 672, 676, 680, 684, 688, 692, 696,
        700, 704, 708, 712, 716, 720, 724, 728, 732, 736, 740, 744, 748, 752, 756, 760, 764, 768, 772, 776, 780, 784, 788, 792, 796,
        800, 804, 808, 812, 816, 820, 824, 828, 832, 836, 840, 844, 848, 852, 856, 860, 864, 868, 872, 876, 880, 884, 888, 892, 896,
        900, 904, 908, 912, 916, 920, 924, 928, 932, 936, 940, 944, 948, 952, 956, 960, 964, 968, 972, 976, 980, 984, 988, 992, 996,
        1000, 1004, 1008, 1012, 1016, 1020, 1023
    }
}
};

#endif

