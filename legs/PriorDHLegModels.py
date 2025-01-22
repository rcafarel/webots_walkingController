import math

from legs.DHLeg import DHLeg
from legs.DHSegment import DHSegment

frontRightLegSegments = []
frontRightLegSegments.append(DHSegment(math.pi ,-167.53507879838534 ,-27.103917802960726 ,0.00130757243978844))
frontRightLegSegments.append(DHSegment(0.01859990518682953 ,167.52870463734263 ,-43.87777697475387 ,-1.577775416269053))
frontRightLegSegments.append(DHSegment(3.1173466205281684 ,8.830906593581252 ,74.924362877956 ,-2.8799685564437043))
frontRightLegSegments.append(DHSegment(0.8110094438745779 ,6.7826862830008565 ,136.184811564699 ,0))

frontRightLeg = DHLeg("frontRightLeg", 39.5, 100.0, 1.0 * math.pi / 4.0, 200.0, 180.0, False, frontRightLegSegments)
