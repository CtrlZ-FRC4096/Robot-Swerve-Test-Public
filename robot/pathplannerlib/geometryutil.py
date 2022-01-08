import wpimath.geometry

def doubleLerp(startVal, endVal, t):
    return startVal + (endVal - startVal) * t

def rotationLerp(startVal, endVal, t):
    return startVal+((endVal-startVal)*t)

def translationLerp(a, b, t):
    return a+((b-a)*t)

def quadraticLerp(a, b, c, t):
    p0 = translationLerp(a, b, t)
    p1 = translationLerp(b, c, t)
    return translationLerp(p0, p1, t)

def cubicLerp(a, b, c, d, t):
    p0 = quadraticLerp(a, b, c, t)
    p1 = quadraticLerp(b, c, d, t)
    return translationLerp(p0, p1, t)