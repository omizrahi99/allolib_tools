from math import log

# Script that quantizes each note to nearest user specified note

# get bpm and file_path from user
bpm = int(input("What BPM did you record at? "))
quantize_note = input(
    "What note would you like to quantize to? (e.g. 1/4, 1/8, 1/16...) ")
file_path = input("What is the file path of your note list? ")

denominator = int(quantize_note.split("/")[1])
p = log(denominator, 2)

string_list = []

# Get file input
my_file = open(file_path)
string_list = my_file.readlines()
my_file.close()


def closest(lst, K):
    return lst[min(range(len(lst)), key=lambda i: abs(lst[i]-K))]


# Create array of all times where user chosen notes occur
time_interval = 60 / (bpm * (2**(p-1)))
times = [0]
for i in range(1, len(string_list)-2):
    times.append(time_interval+times[i-1])

final_arr = []
# Get file input
for line in string_list:
    # get starting value of note
    note_data = line.split()
    if note_data[0] != '#':
        # make starting value of note the closest user-defined note value
        note_data[1] = str(closest(times, float(note_data[1])))
        # join array back into string
        separator = ' '
        final_arr.append(separator.join(note_data))


my_file = open(file_path, "w")
new_file_contents = "\n".join(final_arr)

my_file.write(new_file_contents)
my_file.close()

readable_file = open(file_path)
read_file = readable_file.read()
print(read_file)
