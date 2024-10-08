# TEACHING GOALS

# This will serve as an intro to python. In Lecture 1 we went over some basic python structures, but
# here we actually start to use them to do interesting things for us. Write everything as a function 
# no main(). This encourages students to think of functions as part of a whole. 

# Write a program that takes an integer input from the user and prints double the number to the terminal

# num = int(input("What number should I double?"))
# num = num * 2
# #num *= 2
# print(num)

# # Repeat the above exercise, but now print out a coherent sentence with the answer

# num = int(input("What number should I double?"))
# double = num * 2
# #num *= 2
# # print(f"{num} doubled is {doube}.")
# print(str(num) + " doubled is " + str(double))

# write a function that takes user inputs and returns them as a list. User stops inputting when they enter -1.

# def get_list_numbers():
#     list_nums = []
    
#     while True: # indefinite iteration
#         num = int(input("Please enter a number:"))
        
#         if num == -1: # conditional
#             return list_nums
#         else:
#             list_nums.append(num)
            
# numbers = get_list_numbers()

# print(numbers)        


# # Write a function that takes in a list of numbers and prints out the sum

# def print_sum(num_list: list):
    
#     total = 0
    
#     # for i in range(len(num_list)): # definite iteration
#     #     total += num_list[i]
        
#     # for num in num_list: # you can also loop through elements directly

#     for num in num_list:
#         total += num
        
#     print(str(total))
#     # print(str(sum(num_list))) # there are many built-in functions, consider whether you really need to be writing a function
        
# print_sum([0, 1, 5, 1.1, 54, 4.009])


# def print_sum(num_list: list):
#     total = 0
    
#     # Iterate through elements directly
#     for num in num_list:
#         total += num
        
#     # Print the total sum
#     print(str(total))

# # Call the function with a list of numbers
# print_sum([0, 1, 5, 1.1, 54, 4.009])


# # write a function that returns a boolean of True when the input number is odd

# def is_odd(num: int):
#     if num % 2 != 0:
#         return True
#     else:
#         return False
    
#     return False # this is not really needed

# write a function that takes in a list of integers and returns a list of all the odd numbers

# def find_odds(int_list: list):
    
#     odd_numbers = []
    
#     # for num in int_list:
#     #     if is_odd(num): 
#     #         odd_numbers.append(num)
            
#     for num in int_list:
#         if num % 2 == 0: # modulus
#             continue
#         else:
#             odd_numbers.append(num)
    
#     return odd_numbers

# print(find_odds([1, 4, 5, 9, 10]))

# Write a function that sorts a list of integers in descending order

# def sort(int_list: list):
    
#     for i in range(len(int_list)):
#         for j in range(i+1,len(int_list)):
#             if int_list[j] > int_list[i]:

#                 temp = int_list[i]
#                 int_list[i] = int_list[j] # this does not work, good place to look at debugging!
#                 int_list[j] = temp
                
#                 # temp = int_list[i]
#                 # int_list[i] = int_list[j]
#                 # int_list[j] = temp
                
#                 # int_list[i], int_list[j] = int_list[j], int_list[i]
                
#     return int_list

# print(sort([1, 4, 3, 5, 9, 10, 100]))

# write a function that generates a random 10 digit password

from random import randint
#import random
#import random as rnd
# from random import * -> not recommended

# def generate_password():
#     pw = ""
    
#     for i in range(10):
#         pw += str(randint(0,9))
        
#     # return pw
    
#     return f"{randint(0,1e10):010}"

# print(generate_password())

# write a function that generates a random 10 digit password that is not repeated

# def generate_unique_password(): # this may be a good one for the debugger
#     pw = ""
#     pw_list = []
    
#     while len(pw_list) < 10:
#         random_num = str(randint(0,9))
#         if random_num not in pw_list:
#             pw_list.append(random_num)
#             pw += str(random_num)
        
#     return pw
    
# print(generate_unique_password())

# write a function that reads in a csv file that consists of an array of numbers
# the function must return a list that contains the average of each row

# def read_file(filename: str):
#     row_means = []
    
#     with open(filename) as file:
#             for line in file:
#                 line = line.replace("\n", "")
#                 num_list = line.split(",")
#                 num_list = [int(num) for num in num_list]
#                 row_means.append(sum(num_list)/len(num_list))
#     try:
#         with open(filename) as file:
#             for line in file:
#                 line = line.replace("\n", "")
#                 num_list = line.split(",")
#                 num_list = [int(num) for num in num_list]
#                 row_means.append(sum(num_list)/len(num_list))
#     except: # generic exception
#         pass # not a good idea to just pass
#         print("File not found") # at least we know something happened, but can be more specific
#     # except: FileNotFoundError as fnf_error: # specify specific exceptions
#     #     print(fnf_error)

#     return row_means

# print(read_file("file1.csv"))

# write a function that takes in an integer number between 0 and 5 (inclusive)
# and returns the factorial of that number

def factorial(num: int):
    
    # raising an exception
    if num < 1:
        raise ValueError("The number should be a positive integer. The factorial of a negative number does not exist")
        raise Exception("The number should be a positive integer.")
    
    # assertions are similar to the example above. Not the best method
    # because python sometimes disables them
    assert (num < 1), "The number should be a positive integer."
    
    out = 1
    for i in range(1,num+1):
        out *= i
        
    return out
    
print(factorial(3))