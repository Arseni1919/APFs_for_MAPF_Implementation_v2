import itertools

# Sample list
# lst = [1, 2, 3, 4, 5]
lst = [5]

# Iterate over pairs of consecutive items
for a, b in itertools.pairwise(lst):
    print(a, b)