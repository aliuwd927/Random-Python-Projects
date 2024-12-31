def find_average(nums):
    #your code here
    count_tracker = 0
    
    for num in nums:
        count_tracker += num

    return count_tracker / len(nums)
 
    


print(find_average([5, 7, 3, 7]))