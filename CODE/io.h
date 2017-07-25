/*
 -- ============================================================================
 -- PROJECT NAME	:		Amelia 飞控
 -- FILE_NAME			: 	io.h
 -- DESCRIPTION 	: 	LED和按键
 -- ----------------------------------------------------------------------------
 -- Revision  Date		  Coding_by	 Comment
 -- 0.0.0	  2017/3/9    Maxwell_W	 Maxwell_W
 --	Copyright:	Amelia 使用CC知识共享协议	知识共享署名-相同方式共享 4.0 国际许可协议	发布
 -- ============================================================================
*/

#ifndef	__IO_h__
	#define	__IO_h__
		
	void	led_switch	(unsigned	int	led_switch);	//LED	
	void	bibi(unsigned	int	bibi);	
	unsigned	int	key_read	(void);	//KEY
	void	sys_set	(void);
		
#endif
