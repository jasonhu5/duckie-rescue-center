from autobot_info import AutobotInfo, Distress


a = Distress.NORMAL_OPERATION
print(a)
print(a.value == 0)
print(a == Distress.NORMAL_OPERATION)
print(Distress(1))

b = list()
b.append(2)
if b:
    print(b.pop(0))
print(b)