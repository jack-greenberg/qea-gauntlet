import pandas as pd
import numpy as np

sources = pd.read_csv('sources.csv')
sinks   = pd.read_csv('sinks.csv')

full = ""

for idx, row in sources.iterrows():
    x = str(abs(round(row[0], 3)))
    x_sign = np.sign(row[0])
    x_sign_str = '+' if x_sign == 1 or x_sign == 0 else '-'
    x_term = "(x - {x})^{{2}}".format(x=x)

    y = str(abs(round(row[1], 3)))
    y_sign = np.sign(row[1])
    y_sign_str = '+' if y_sign == 1 or y_sign == 0 else '-'
    y_term = "(y - {y})^{{2}}".format(y=y)

    full_term = " - ln \\sqrt{{{xterm} %2b {yterm}}}".format(xterm=x_term, yterm=y_term)
    full = full + full_term

for idx, row in sinks.iterrows():
    x = str(abs(round(row[0], 3)))
    x_sign = np.sign(row[0])
    x_sign_str = '+' if x_sign == 1 or x_sign == 0 else '-'
    x_term = "(x - {x})^{{2}}".format(x=x)

    y = str(abs(round(row[1], 3)))
    y_sign = np.sign(row[1])
    y_sign_str = '+' if y_sign == 1 or y_sign == 0 else '-'
    y_term = "(y - {y})^{{2}}".format(y=y)

    full_term = " + ln \\sqrt{{{xterm} - {yterm}}}".format(xterm=x_term, yterm=y_term)
    full = full + full_term

print(full)
