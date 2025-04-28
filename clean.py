with open("janikhurd.csv", "r") as file:
    data = file.read()

data = data.replace("\t", ",")

print(data)

with open("janikhurd.csv", "w") as file:
    file.write(data)
