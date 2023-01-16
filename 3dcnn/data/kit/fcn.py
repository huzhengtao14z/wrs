from keras.models import Sequential, Model, load_model
from keras.layers import Dense, Dropout, Activation, Flatten
from keras.optimizers import SGD
from keras.datasets import mnist
from sklearn.model_selection import train_test_split
from keras.layers import Input, concatenate
import numpy as np
import copy
import pickle
import os
import open3d as o3d
# import open3d.geometry as o3dg
import vision.depth_camera.pcd_data_adapter as vdda
import cv2
import tensorflow as tf
from keras.callbacks import TensorBoard

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'
model = Sequential()

model.add(Dense(64, input_shape=(500,3)))  # 输入层
model.add(Activation('tanh'))
# model.add(Dropout(0.5))  # 50% dropout

model.add(Dense(128))  # 隐藏层， 500
model.add(Activation('tanh'))
# model.add(Dropout(0.5))  # 50% dropout

model.add(Dense(256))  # 隐藏层， 500
model.add(Activation('tanh'))
# model.add(Dropout(0.5))  # 50% dropout

model.add(Dense(512))  # 隐藏层， 500
model.add(Activation('tanh'))
model.add(Dropout(0.5))  # 50% dropout

model.add(Dense(2048, activation='relu'))  # 输出结果， 2
model.add(Dropout(0.4))
model.add(Dense(512, activation='relu'))
model.add(Dropout(0.4))
model.add(Flatten())
model.add(Dense(3, activation='linear'))

sgd = SGD(lr=0.001, momentum=0.9, nesterov=True)  # 设定学习效率等参数

# lossfunction = tf.keras.metrics.RootMeanSquaredError
# model.compile(loss=tf.keras.losses.MeanSquaredError(), optimizer='sgd')  # 使用交叉熵作为loss
model.compile(
    optimizer='sgd',
    loss='mse',
    metrics=[tf.keras.metrics.RootMeanSquaredError()])

with open('pairtial/com_v2.pickle', 'rb') as f:
    comdir = pickle.load(f)
this_dir, this_filename = os.path.split(__file__)
objmodeldir = 'C:/Users/GL65/Documents/GitHub/wrs/3dcnn/kit_model'
plydir = 'C:/Users/GL65/Documents/GitHub/wrs/3dcnn/data/kit/pc_500'
namelist = os.listdir(plydir)
data_x = []
data_y = []
for name in namelist:
    objname = name.split('.')[0]
    data_y.append(comdir[objname]*1000)
    pcd = o3d.io.read_point_cloud(plydir + '/' + name)
    # dc_pcd = pcd.uniform_down_sample(100)
    data_x.append(vdda.o3dpcd_to_parray(pcd)*1000)


data_x = np.asarray(data_x)
data_y = np.asarray(data_y)
x, test_x, y, test_y = train_test_split(data_x, data_y, test_size=0.2, random_state=40)


# X_train = np.array(x)
# X_test = test_x
# Y_train = (np.arange(2) == np.array(y)[:, np.newaxis]).astype(int)
# Y_test = (np.arange(2) == np.array(test_y)[:, np.newaxis]).astype(int)
# x = x[:,np.newaxis]
# x = np.expand_dims(x, axis = 3)
y = np.expand_dims(y, axis = 2)
tensorboard_callback = tf.keras.callbacks.TensorBoard(
    log_dir='my_log_dir',  # 日志文件的储存位置
    histogram_freq=1,      # 每 histogram_freq 轮之后记录激活直方图
    embeddings_freq=1,     # 每 histogram_freq 轮之后记录词嵌入
)


model.fit(x, y, batch_size=32, epochs=100, validation_split=0.2, callbacks=[tensorboard_callback])

# model = load_model('fcn-com.h5')
model.save('fcn-com.h5')
print("test set")
# 开始预测
# scores = model.evaluate(X_test, Y_test, batch_size=32, verbose=1)
# print("")
# print("The test loss is %f" % scores)
result = model.predict(test_x, batch_size=8, verbose=1)
print(result[:10])
print("---")
print(test_y[:10])
# result_max = np.argmax(result, axis=1)
# test_max = np.argmax(Y_test, axis=1)

# result_bool = np.equal(result_max, test_max)
# true_num = np.sum(result_bool)
# print("")
# print("The accuracy of the model is %f" % (true_num / len(result_bool)))