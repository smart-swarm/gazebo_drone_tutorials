#!/usr/bin/env python

from example import Example

if __name__ == "__main__":
    
    example = Example()
    example.set_rate(50)

    while not example.is_shutdown():
        example.process()
        example.sleep()
        
        