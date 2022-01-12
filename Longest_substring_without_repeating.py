```
Given a string s, find the length of the longest substring without repeating characters.
Input: s = "abcabcbb"
Output: 3
Explanation: The answer is "abc", with the length of 3.
```

class Solution:
    def lengthOfLongestSubstring(self, s: str) -> int:
        seen = []
        maximum=0
        for i in s: 
            if i in seen:
                maximum = max(maximum,len(seen))
                seen = seen[seen.index(i)+1:]
            seen.append(i)
        maximum = max(maximum,len(seen))
        return maximum
