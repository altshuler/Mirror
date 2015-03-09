#include <string.h>
#include <errno.h>
#include "itoa.h"

void reverse(char s[]);
 
/* reverse:  reverse string s in place */
void reverse(char s[])
{
	int i, j;
	char c;

	for (i = 0, j = strlen(s)-1; i<j; i++, j--) {
		c = s[i];
		s[i] = s[j];
		s[j] = c;
	}
}


/* itoa:  convert n to characters in s */
 char *itoa(int n, char s[], int radix)
 {
     int i, sign;
 
	 if (radix > 36 || radix <= 1)
	 {
	   //__set_errno(EDOM);
	   return 0;
	 }
 
 	 sign = (radix == 10 && n < 0);  /* record sign */
	 if (sign)
         n = -n;          /* make n positive */
     i = 0;
     do {       /* generate digits in reverse order */
	 	if (10<radix)
         s[i++] = (n % radix) + (((n % radix)<10) ?  '0' : ('a' - 10));   /* get next digit */
		else
			s[i++] = n % radix + '0';	 /* get next digit */
     } while ((n /= radix) > 0);     /* delete it */
     if (sign < 0)
         s[i++] = '-';
     s[i] = '\0';
     reverse(s);
	 return s;
 }


