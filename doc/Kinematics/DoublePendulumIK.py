import numpy as np

def jacobian(theta):
	theta1 = theta[0]
	theta2 = theta[1]
	costheta1 = np.cos(theta1)
	sintheta1 = np.sin(theta1)
	coscombo = np.cos(theta1+theta2)
	sincombo = np.sin(theta1+theta2)

	return np.array([[costheta1+coscombo, sintheta1+sincombo],[coscombo,sincombo]])

def position(theta):
	theta1 = theta[0]
	theta2 = theta[1]
	costheta1 = np.cos(theta1)
	sintheta1 = np.sin(theta1)
	coscombo = np.cos(theta1+theta2)
	sincombo = np.sin(theta1+theta2)

	return np.array([sincombo+sintheta1,-coscombo-costheta1])

J = []
X = np.array([0,0])
A = np.array([0,0])

# dA2

X_goal = np.array([1,0.4])

for i in range(100):
	X = position(A)
	dX = X_goal - X
	if(np.linalg.norm(dX) < 0.0001):
		break
	J = jacobian(A)

	if(np.linalg.det(J) != 0):
		dA = np.dot(np.linalg.inv(J), dX)
	else:
		dA = dX
	print "error = ", np.linalg.norm(dX), "Angles = ", A, "Iterations = ", i+1

	A = dA + A

print "Goal result = ", X_goal
print "Final result = ", X
print "Error = ", X_goal - X
print "Final angles = ", A
