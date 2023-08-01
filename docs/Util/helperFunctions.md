# Utility Functions

The utility functions module provides several useful functions for working with rotations and quaternions. These functions are used in various parts of the codebase to manipulate and convert rotation representations.

## Description

The utility functions module provides several functions for working with rotations and quaternions. These functions are used in various parts of the codebase to manipulate and convert rotation representations.

## Functions

- `sendData(rotation, centre, sock, ip, port)`: Sends data to Unity with rotation and center coordinates.
- `radToDeg(rad)`: Converts radians to degrees.
- `normaliseQuaternion(q)`: Normalizes a quaternion.
- `multiplyQuaternions(q0, q1)`: Multiplies two quaternions together.
- `rotationMatrixToQuaternion(rotation)`: Converts a 3x3 rotation matrix to a quaternion.
- `quaternionToEuler(rotation)`: Converts a quaternion to Euler angles (roll, pitch, yaw).
- `quaternionInverse(q)`: Computes the inverse of a quaternion.
