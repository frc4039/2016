import Tkinter as tk
import os.path

file_name = '03-17/match{ID}'
shot_counter = 0
hit_counter = 0
average = 0
matchID = 0
location = {'q':0, 'w':0, 'e':0, 'a':0, 's':0, 'd':0, 'z':0, 'x':0, 'c':0}

def printResults():
    global file_name, matchID, location, shot_counter, average
    while os.path.isfile(file_name.format(ID=matchID)):
        matchID += 1
    text.insert('end', ' printing results to file %s\n'%file_name.format(ID=matchID))
    with open(file_name.format(ID=matchID), 'w') as f:
        f.write('''Overall ---------------
Shots taken: %d
Shots hit: %d
Average: %0.1f%%

Missed Shots ----------
High:
    left:   %d
    middle: %d
    right:  %d
Middle:
    left:   %d
    right:  %d
Low:
    left:   %d
    middle: %d
    right:  %d
'''%(shot_counter, location['s'], average*100.0,
     location['q'], location['w'], location['e'],
     location['a'], location['d'],
     location['z'], location['x'], location['c'])
                )
    #reset values
    text.insert('end', ' resetting score ')
    for i in location.keys():
        location[i] = 0
    shot_counter = 0
    average = 0


def onKeyPress(event):
    global shot_counter, location, average
    shot_counter += 1
    if event.keysym == 'Escape':
        text.insert('end', ' resetting score ')
        for i in location.keys():
            location[i] = 0
        shot_counter = 0
        average = 0
    elif event.char == 's':
        location['s'] += 1
        text.insert('end', ' hit!')
        average = float(location['s'])/float(shot_counter)
        text.insert('end', ' average: %f\n'%(average))
    elif event.char == 'p':
        printResults()
    else:
        try:
            location[event.char] += 1
        except:
            pass
        text.insert('end', ' miss :( ')
        average = float(location['s'])/float(shot_counter)
        text.insert('end', 'average: %f\n'%(average))



root = tk.Tk()
root.geometry('300x200')

text = tk.Text(root, background='white', foreground='black', font=('Comic Sans MS', 12))
text.pack()

root.bind('<KeyPress>', onKeyPress)
root.mainloop()


