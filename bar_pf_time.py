import matplotlib.pyplot as plt

name_list = ['0', '1', '2', '3', '4']
rm_list = [9.3, 4.9, 7.9, 10.5, 3.8]
gpu_list = [33.4, 18.1, 25.9, 27.5, 14.3]
cddt_list = [13.7, 5.8, 10.2, 13.3, 5.7]
# int
# rm_list = list(map(int, rm_list))
# gpu_list = list(map(int, gpu_list))
# cddt_list = list(map(int, cddt_list))

def autolabel(rects):
    for rect in rects:
        height = rect.get_height()
        plt.text(rect.get_x() +rect.get_width()/2.-0.1, height + 0.1, '%s' % int(round(height)))

x = list(range(5))
total_width, n = 0.9, 3
width = total_width / n
rm_bar = plt.bar(x, rm_list, width=width, label='RM', fc='y')
for i in range(len(x)):
    x[i] = x[i] + width
cddt_bar = plt.bar(x, cddt_list, width=width, label='CDDT', fc='r')
for i in range(len(x)):
    x[i] = x[i] + width
gpu_bar = plt.bar(x, gpu_list, width=width, label='RM GPU', fc='b')
autolabel(rm_bar)
autolabel(cddt_bar)
autolabel(gpu_bar)
# for a, b in zip(name_list, rm_list):
    # plt.text(a, b+0.05, '.2f' % b, ha='center', va='bottom')
    # plt.text(a, b+0.05, '???')
plt.xlabel("Jetson Mode")
plt.ylabel("iters per second")
plt.legend()
plt.show()
