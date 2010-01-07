

      subroutine parser (string,field)
!
      integer   i,j
      character char*1,string*(*),field*(*)
      logical   advance
!
      i = 0
      j = 0
      field = ' '
      advance = .true.
!
      do while ( advance )
         i = i + 1
         char = string(i:i)
         if ( char.ne.' ' .and. char.ne.',' ) then
            j = j + 1
            field(j:j) = char
         else 
            string(i:i) = ' '
            advance = j.eq.0 .and. i.lt. len(string)
         end if
      end do
!
      string = string(i+1:len(string))
!
      return
      end
