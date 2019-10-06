number = 0
while True:
    print("Select the mode")
    print("0: direct mode (no reset, not recommended")
    print("1: test code mode")
    print("2: reset position and start playing")
    print("3: look at current position and decide")

    number = raw_input("->")

    if int(number) >=0 and int(number) <= 3:
        switch = int(number)
        break
    else:
        print("wrong value")

print(number)