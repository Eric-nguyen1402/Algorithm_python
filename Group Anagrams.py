from collections import defaultdict

class Solution(object):
    def groupAnagrams(self, strs):
        """
        :type strs: List[str]
        :rtype: List[List[str]]
        """
        result=defaultdict(list) # mapping charCount to list of anagrams
        for s in strs:
          count = [0] * 26 # a....z
          for c in s:
            count[ord(c) - ord("a")] += 1
          result[tuple(count)].append(s)
        return result.values()


strs = ["eat","tea","tan","ate","nat","bat"]
group = Solution()
group.groupAnagrams(strs)
