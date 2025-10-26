import random, string

chars = string.ascii_letters + string.digits

print("".join(random.choice(chars) for i in range(16)))