import os

myscript_path = os.getcwd()

# classes_lst = ['ball', 'book', 'bottle', 'cat', 'cell phone', 'chair', 'clothes', 'cup', 'dog', 'glasses', 'jewelry', 'paper', 'pen', 'person',
#            'power strip', 'pet fece', 'remote controller', 'shoes' , 'socks' , 'table' , 'toilet' , 'trash can', 'potted plant' , 'weight scale']
# class_dict = {str(i) : classes_lst[i] for i in range(0, len(classes_lst))}

#classes_lst = ['pen', 'wire', 'shoes', 'bottle', 'trash can', 'potted plant', 'weight scale',
#               'lamp base', 'cat', 'dog', 'clothes', 'person', 'hair', 'water', 'cluster_hair']

classes_lst = ['bottle' , 'shoes' , 'wire']
class_dict = {str(i): classes_lst[i] for i in range(0, len(classes_lst))}


def path():
    darknet_path = '/'.join(myscript_path.split('/')
                            [:-1])  # /home/shuo.han/darknet
    # /home/shuo.han/darknet/scripts/VOCdevkit/VOC2012
    path = darknet_path + '/scripts/VOCdevkit/VOC2012'
    return path


def darknet_path():
    darknet_path = '/'.join(myscript_path.split('/')
                            [:-1])  # /home/shuo.han/darknet
    return darknet_path


def classes():
    classes = [i for i in range(0, len(classes_lst))]
    classes = [str(i) for i in classes]

    return classes


if __name__ == '__main__':
    cla = classes()
    print(cla)
    print('classes unmber is:', len(cla))
