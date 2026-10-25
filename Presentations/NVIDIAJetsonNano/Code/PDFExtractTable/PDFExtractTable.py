# Author: Ardit Sulce, Automate Everything with Python, Udemy
# Course URL: https://www.udemy.com/course/automate-everything-with-python/

import tabula

table = tabula.read_pdf('weather.pdf', pages=1)

print(type(table[0]))

table[0].to_csv('output.csv', index=None)