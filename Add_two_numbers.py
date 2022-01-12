```
You are given two non-empty linked lists representing two non-negative integers. The digits are stored in reverse order, and each of their nodes contains a single digit. Add the two numbers and return the sum as a linked list.

You may assume the two numbers do not contain any leading zero, except the number 0 itself.
Input: l1 = [2,4,3], l2 = [5,6,4]
Output: [7,0,8]
Explanation: 342 + 465 = 807.
```

# Definition for singly-linked list.
# class ListNode:
#     def __init__(self, val=0, next=None):
#         self.val = val
#         self.next = next
class Solution:
    def addTwoNumbers(self, l1, l2, a = 0):
        sum = l1.val + l2.val + a
        a = sum // 10
        res = ListNode(sum % 10 ) 
        
        if l1.next or l2.next or a:
            if not l1.next :
                l1.next = ListNode(0)
            if not l2.next:
                l2.next = ListNode(0)
            res.next = self.addTwoNumbers(l1.next,l2.next,a)
        return res
