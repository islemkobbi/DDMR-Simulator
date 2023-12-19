#    This file was created by
#    MATLAB Deep Learning Toolbox Converter for TensorFlow Models.
#    29-May-2023 16:41:16

import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers

def create_model():
    input = keras.Input(shape=(None,6))
    fc = layers.Dense(72, name="fc_")(input)
    relu = layers.ReLU()(fc)
    relu_lstm_input = relu
    lstm = layers.LSTM(75, name='lstm_', activation='tanh', recurrent_activation='sigmoid', return_sequences=True, return_state=False)(relu_lstm_input)
    dropout = layers.Dropout(0.500000)(lstm)
    fc_1 = layers.Dense(72, name="fc_1_")(dropout)
    relu_2 = layers.ReLU()(fc_1)
    fc_3 = layers.Dense(6, name="fc_3_")(relu_2)
    softmax = layers.Softmax()(fc_3)
    loss1 = softmax

    model = keras.Model(inputs=[input], outputs=[loss1])
    return model
