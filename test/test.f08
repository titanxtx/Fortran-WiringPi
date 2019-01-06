program test
use wiringpi
implicit none

 interface
	subroutine sleep(val) bind(C,name="sleep")
		use iso_c_binding,only:c_int
		integer(kind=c_int),intent(in),value::val
	end subroutine 
 end interface

integer(kind=c_int)::w
logical::m=.true.

w=wiringPiSetup()
call pinMode(7,output)

do while(.true.)
	if(m .eqv. .true.) then
		call digitalwrite(7,high)
		m=.false.
	else
		call digitalwrite(7,low) 
		m=.true.
	end if
	call sleep(2)
end do
end program test
