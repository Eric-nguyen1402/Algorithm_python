```
Given a signed 32-bit integer x, return x with its digits reversed. If reversing x causes the value to go outside the signed 32-bit integer range [-2^31, 2^31 - 1], then return 0.
Assume the environment does not allow you to store 64-bit integers (signed or unsigned).
Input: x = 123
Output: 321
```
class Solution(object):
    def reverse(self, x):
        if x <= -2**31 or x >= 2**31-1 :
            return 0
        else: 
            if x>=0:
                a = int(str(x)[::-1])
            if x<0:
                a = -1*int(str(x*-1)[::-1])
            if a <= -2**31 or a >= 2**31-1 :    
                return 0
            return a
