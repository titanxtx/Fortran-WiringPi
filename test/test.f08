!MIT License

!Copyright (c) 2019 Joshua Oliva

!Permission is hereby granted, free of charge, to any person obtaining a copy
!of this software and associated documentation files (the "Software"), to deal
!in the Software without restriction, including without limitation the rights
!to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
!copies of the Software, and to permit persons to whom the Software is
!furnished to do so, subject to the following conditions:

!The above copyright notice and this permission notice shall be included in all
!copies or substantial portions of the Software.

!THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
!IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
!FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
!AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
!LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
!OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
!SOFTWARE.
program test
use wiringpi
implicit none

 interface
	subroutine sleep(val) bind(C,name="sleep")
		use iso_c_binding,only:c_int
		integer(kind=c_int),intent(in),value::val
	end subroutine sleep
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
