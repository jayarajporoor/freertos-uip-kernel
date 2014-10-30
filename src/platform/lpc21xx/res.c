#include <res.h>
#include <klog.h>

static uint32 port0;
static uint32 port1;
static uint32 fiq;
static uint32 vic_vect;

uint32 res_port0_acquire_(const char *user, uint32 pins)
{
	if( (port0 & pins) != 0UL)
	{
		klog_error( "%s:%d> Resource conflict - Req by %s for port0 pins %lx.  Alloc bitmap: %lx"
				   __FILE__
				  ,__LINE__
		          , user
		          , pins
		          , port0 );
		return 0; 
	}
	port0 |= pins;
	return pins;		
}
 
uint32 res_port0_release_(const char *user, uint32 pins)
{
	port0 &= (~pins);	
	return 0;
}

uint32 res_port1_acquire_(const char *user, uint32 pins)
{
	if( (port1 & pins) != 0UL)
	{
		klog_error( "%s:%d> Resource conflict - Req by %s for port1 pins %lx.  Alloc bitmap: %lx"
				  , __FILE__
				  , __LINE__
		          , user
		          , pins
		          , port1 );
		return 0; 
	}
	port1 |= pins;
	return pins;				
} 

uint32 res_port1_release_(const char *user, uint32 pins)
{
	port1 &= (~pins);
	return 0;	
}

uint32 res_fiq_acquire_(const char *user)
{
	if( fiq != 0UL)
	{
		klog_error( "%s:%d> Resource conflict - Req by %s for fiq. Already allocated."
				  , __FILE__, __LINE__
		          , user );
		return 0; 
	}
	fiq=1UL;
	return 1;			
}

uint32 res_fiq_release_(const char *user)
{
	fiq = 0UL;
	return 0;
}

uint32 res_vic_vect_acquire_(const char *user, uint32 vect)
{
	uint32 mask = (1UL << vect);
	if(vect > RES_VIC_VECT_MAX)
	{
		klog_error( "%s:%d> Req by %s for invalid vic vect - %d (must be < %d)"
				  , __FILE__, __LINE__
		          , user
		          , vect
		          , RES_VIC_VECT_MAX );
		return 0; 		
	}
	
	if( (vic_vect & mask) != 0UL)
	{
		klog_error( "%s:%d> Resource conflict - Req by %s for vic vect %lx.  Alloc bitmap: %lx"
				  , __FILE__, __LINE__
		          , user
		          , vect
		          , vic_vect );
		return 0; 
	}
	vic_vect |= mask;
	return vect;				
	
}

uint32 res_vic_vect_acquire_any_(const char *user)
{
	int i;
	uint32 mask;
	
	for(i=0;i<RES_VIC_VECT_MAX;i++)
	{
		mask = (1UL << i);
		if( (vic_vect & mask) == 0UL)
		{
			vic_vect |= mask;
			return i; 
		}
	}
	
	klog_error( "%s:%d> Resource unavailable - Req by %s for any free vic_vect. None available."
				  , __FILE__, __LINE__, user );
	return 0;					
}

uint32 res_vic_vect_release_(const char *user, uint32 vect)
{
	uint32 mask_ = ~(1UL << vect);
	vic_vect &= mask_;
	return vect;
}
