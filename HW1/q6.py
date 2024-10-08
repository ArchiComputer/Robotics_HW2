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
        pass # your code here
        
    def __str__(self):
        pass # your code here
        
square = Square(4)
print(square)
print("area:", square.area())