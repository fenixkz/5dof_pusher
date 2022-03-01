import ast



f = open("qtable.txt", "r")
cont = f.read()
dict = ast.literal_eval(cont)

state = 2229262828
actions = range(10)
q = [dict.get((state, a), 0.0) for a in actions]
print(q.index(max(q)))
