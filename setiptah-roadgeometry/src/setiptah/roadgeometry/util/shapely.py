from shapely import LineString


def split_line_string(line_string: LineString, t: float) -> tuple[LineString, LineString]:
    before, after = [], []
    acc = 0.
    edges = iter(zip(line_string.coords[:-1], line_string.coords[1:]))

    while True:
        if t <= acc:
            break

        try:
            pt1, pt2 = edge = next(edges)
            line_string_ = LineString(edge)

            d = line_string_.length

            if t < acc + d:
                pt3_ = line_string_.interpolate(t - acc)
                pt3 = pt3_.x, pt3_.y
                before.append((pt1, pt3))
                after.append((pt3, pt2))
            else:
                before.append(edge)
            acc += d

        except StopIteration:
            break

    after.extend(edges)

    return LineString(to_points(before)), LineString(to_points(after))


def crop_line_string(line_string: LineString, start: float, end: float) -> LineString:
    assert start <= end
    _, suffix = split_line_string(line_string, start)
    result, _ = split_line_string(suffix, end - start)
    return result


def to_points(seg):
    points = []
    for x, y in seg:
        if len(points) <= 0:
            points.append(x)
        points.append(y)
    return points
