# Collaborators : None



# Question 2 - (a)

def sequence(maximum_sum):
    list = []
    current_sum = 0
    k = 1

    while current_sum <= maximum_sum:
        term = k**2 + 1
        list.append(term)
        current_sum += term
        k += 1
    
    return list

print(sequence(22))




# Question 2 - (b)

import math

def smallest_multiple(list):

    # calculate LCM of two numbers
    def LCM(a, b):
        return abs(a * b) // math.gcd(a, b)
    
    # iteratively find LCM in the list
    result = list[0]

    for num in list[1:]:
        result = LCM(result, num)
    
    return result

LCM_list = smallest_multiple([2, 4, 7])
print(LCM_list)





# Question 3

from scipy.integrate import quad
import numpy as np
import matplotlib.pyplot as plt

def inside(x):

    return x**2 / np.sqrt(1 + x + x**2)

def f(a):

    integralValue = quad(inside, -2, a)
    return integralValue

aValue = np.arange(-2, 2.01, 0.01)
fValues = [f(a) for a in aValue]

plt.figure(figsize=(8, 6))
plt.plot(aValue, fValues)
plt.title(r'The integral $f(a)$ for various values of $a$', fontsize=20)
plt.xlabel(r'Upper bound, $a$', fontsize=20)
plt.ylabel(r'Integral value, $f(a)$', fontsize=20)
plt.grid(True)
plt.axhline(0, color='black',linewidth=0.3)
plt.axvline(0, color='black',linewidth=0.3)
plt.legend()

plt.show()
plt.savefig('Question3.png')





# Question 4

import numpy as np
from itertools import permutations

def check_prime(number):

    if number < 2:
        return False
    
    for i in range(2, int(np.sqrt(number)) + 1):

        if number % i == 0:
            return False
        
    return True

def check_circular_prime(number):
    string_numder = str(number)
    return all(check_prime(int(''.join(p))) for p in permutations(string_numder))

def cprimes_list(N):
    circular_primes = set()
    natural_number = 2
    while len(circular_primes) < N:
        if check_circular_prime(natural_number):
            circular_primes.add(natural_number)
        natural_number += 1
    return np.array(sorted(circular_primes))

print(cprimes_list(20))





# Question 5

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sklearn.linear_model import LinearRegression
import matplotlib

matplotlib.use('Agg')  # Switch to a non-interactive backend

# load q5_data
q5_data = np.loadtxt('q5.csv', delimiter=',')

# features (X) and target (y) and get coefficients and intercept
X = q5_data[:, :2]  
Y = q5_data[:, 2]   

model = LinearRegression()
model.fit(X, Y)

a, b = model.coef_
c = model.intercept_

print(f"Coefficients 'a', 'b' and intercept 'c' : a = {a}, b = {b}, c = {c}")

# Generate predictions
Y_prediction = model.predict(X)

# Create 3D scatter plot
figure = plt.figure(figsize=(10, 8))
Axes = figure.add_subplot(111, projection='3d')

Axes.scatter(X[:, 0], X[:, 1], Y, c='r', marker='x', label='Data')
Axes.scatter(X[:, 0], X[:, 1], Y_prediction, c='b', marker='^', label='Estimates')

# Set labels and title
Axes.set_xlabel('X')
Axes.set_ylabel('Y')
Axes.set_zlabel('Z')

# Add legend
Axes.legend(loc='upper right', fontsize='medium')

# Show the plot
plt.show()
plt.savefig('Question6.png')




# Question 6

class Rectangle:
    def __init__(self, width: int, height: int):
        self.width = width
        self.height = height

    def __str__(self):
        return f"rectangle {self.width}x{self.height}"

    def area(self):
        return self.width * self.height
    
class Square(Rectangle):
    def __init__(self, width):
        super().__init__(width, width)
        
    def __str__(self):
        return f"square {self.width}x{self.width}"
    
    def area(self):
        return self.width * self.width
        
square = Square(4)
print(square)
print("area:", square.area())




# Question 7

import random

class WordGame():
    def __init__(self, rounds: int):
        self.wins1 = 0
        self.wins2 = 0
        self.rounds = rounds

    def round_winner(self, player1_word: str, player2_word: str):
        # determine a random winner
        return random.randint(1, 2)

    def play(self):
        print("Word game:")
        for i in range(1, self.rounds+1):
            print(f"round {i}")
            answer1 = input("player1: ")
            answer2 = input("player2: ")

            if self.round_winner(answer1, answer2) == 1:
                self.wins1 += 1
                print("player 1 won")
            elif self.round_winner(answer1, answer2) == 2:
                self.wins2 += 1
                print("player 2 won")
            else:
                print("it's a tie")
                pass # it's a tie

        print("game over, wins:")
        print(f"player 1: {self.wins1}")
        print(f"player 2: {self.wins2}")
        



class RockPaperScissors(WordGame):
    def __init__(self, rounds: int):
        super().__init__(rounds)

    def round_winner(self, player1_input: str, player2_input: str):

        valid_input = {'rock', 'paper', 'scissors'}

        if player1_input not in valid_input and player2_input not in valid_input:
            return 0

        if player2_input not in valid_input:
            return 1 

        if player1_input not in valid_input:
            return 2  

        outcomes = {
            ('rock', 'scissors'): 1,
            ('scissors', 'rock'): 2,
            ('paper', 'rock'): 1,
            ('rock', 'paper'): 2,
            ('scissors', 'paper'): 1,
            ('paper', 'scissors'): 2
        }
                
        return outcomes.get((player1_input, player2_input), 0)  

# example
game = RockPaperScissors(3)
game.play()