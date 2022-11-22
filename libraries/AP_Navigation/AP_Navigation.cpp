#include "AP_Navigation.h"

// Set destination location and relative turn point
// negative turn point turns before waypoint, positive turns after
void AP_Navigation::set_destination(const struct Location &dest, const struct Location &next_dest, TURN_TYPE type, float turn_radius, float turn_dist)
{
    _prev_dest = _dest;
    _dest = dest;
    _next_dest = next_dest;

    const float acceptance_radius = turn_radius;

    // could check new dest is same as old next destination

    // set turn type
    _dest_turn_point = turn_dist;
    _turn_type = type;

    // Convert to vectors
    const Vector2F BA = _dest.get_distance_NE_ftype(_prev_dest);
    const Vector2F BC = _dest.get_distance_NE_ftype(_next_dest);
    const Vector2F BA_norm = BA.normalized();
    const Vector2F BC_norm = BC.normalized();

    // half angle between vectors
    const float angle = acosF(BA_norm.dot(BC_norm)) * 0.5;

    // Need fillet distance in all cases
    // this is the distance from the center of a circle of WP radius and the destination point
    const float fillet_center_dist = turn_radius / sinf(angle);
    const float fillet_tangent_dist = turn_radius / tanf(angle);

    if (((_turn_type == TURN_TYPE::TEARDROP) && (fillet_center_dist <= (acceptance_radius + turn_radius))) ||
        ((_turn_type == TURN_TYPE::LATE) && (-_dest_turn_point <= fillet_tangent_dist))) {
        // no need for extra turns, simple fillet results in vehicle travelling within acceptance radius
        // turn point is too far before WP B, fillet is optimal
        _turn_type = TURN_TYPE::FILLET;
        _dest_turn_point = 0.0;
    }

    _next_path.origin = _dest;

    switch(_turn_type) {
        case TURN_TYPE::FILLET: {
            // fillet is single turn tangent to both paths
            _next_path.num_segments = 2;

            // calculate start and end points
            const Vector2F turn_start = BA_norm * fillet_tangent_dist;
            const Vector2F turn_end = BC_norm * fillet_tangent_dist;

            // straight upto fillet turn
            _next_path.segments[0].type = DUBINS_PATH::SEGMENT::STRAIGHT;
            _next_path.segments[0].path.straight.start = BA;
            _next_path.segments[0].path.straight.vec = -BA - turn_start;

            // fillet turn
            _next_path.segments[1].type = is_positive(Vector2F{BA.y, -BA.x}.dot(BC)) ? DUBINS_PATH::SEGMENT::LEFT : DUBINS_PATH::SEGMENT::RIGHT;
            _next_path.segments[1].path.turn.center = (BA_norm + BC_norm).normalized() * fillet_center_dist;
            _next_path.segments[1].path.turn.start_angle = (turn_start - _next_path.segments[1].path.turn.center).angle();
            _next_path.segments[1].path.turn.end_angle = (turn_end - _next_path.segments[1].path.turn.center).angle();

            break;
        }

        case TURN_TYPE::TEARDROP: {
            // Teardrop fillet is three turns such that the vehicle always travels within acceptance radius
            // either LRL or RLR, symmetrically
            _next_path.num_segments = 4;

            Vector2F BA_normal {BA.y, -BA.x};
            if (BA_normal.dot(BC) > 0.0) {
                BA_normal *= -1.0;
                _next_path.segments[1].type = DUBINS_PATH::SEGMENT::RIGHT;
                _next_path.segments[2].type = DUBINS_PATH::SEGMENT::LEFT;
                _next_path.segments[3].type = DUBINS_PATH::SEGMENT::RIGHT;
            } else {
                _next_path.segments[1].type = DUBINS_PATH::SEGMENT::LEFT;
                _next_path.segments[2].type = DUBINS_PATH::SEGMENT::RIGHT;
                _next_path.segments[3].type = DUBINS_PATH::SEGMENT::LEFT;
            }

            Vector2 BC_normal {BC.y, - BC.x};
            if (BC_normal.dot(BA) > 0.0) {
                BC_normal *= -1.0;
            }

            // center of main turn is on the bisector
            _next_path.segments[2].path.turn.center = (BA_norm + BC_norm).normalized() * (turn_radius + acceptance_radius);

            // temporary point on same BA normal as segment 2 center, one turn radius away from line BA
            const Vector2F temp_point = BA_norm * _next_path.segments[2].path.turn.center.dot(BA_norm) + BA_normal * turn_radius;

            // center of first turn
            _next_path.segments[1].path.turn.center = temp_point + BA_norm * sqrtf(sq(2.0*turn_radius) - (temp_point - _next_path.segments[2].path.turn.center).length_squared());

            // first tangent point is projection of first center
            const Vector2F tangent1 = BA_norm * _next_path.segments[1].path.turn.center.dot(BA_norm);

            // straight segment ends as that tangent point
            _next_path.segments[0].type = DUBINS_PATH::SEGMENT::STRAIGHT;
            _next_path.segments[0].path.straight.start = BA;
            _next_path.segments[0].path.straight.vec = -BA - tangent1;

            // second tangent point is midway between first and second centers
            const Vector2F tangent2 = _next_path.segments[2].path.turn.center + (_next_path.segments[1].path.turn.center - _next_path.segments[2].path.turn.center) * 0.5;

            // first turn start and end points can now be calculated
            _next_path.segments[1].path.turn.start_angle = (tangent1 - _next_path.segments[1].path.turn.center).angle();
            _next_path.segments[1].path.turn.end_angle =  (tangent2 - _next_path.segments[1].path.turn.center).angle();

            // second tangent point is also start of next turn
            _next_path.segments[2].path.turn.start_angle = (tangent2 - _next_path.segments[2].path.turn.center).angle();

            // use first tangent point to get last
            const Vector2F tangent4 = BC_norm * tangent1.length();

            // final center point is calculated from that tangent
            _next_path.segments[3].path.turn.center = tangent4 + BC_normal.normalized() * turn_radius;

            // again tangent is midway between centers
            const Vector2F tangent3 = _next_path.segments[2].path.turn.center + (_next_path.segments[3].path.turn.center - _next_path.segments[2].path.turn.center) * 0.5;

            // fill in remaining start and stop angles
            _next_path.segments[2].path.turn.end_angle = (tangent3 - _next_path.segments[2].path.turn.center).angle();
            _next_path.segments[3].path.turn.start_angle = (tangent3 - _next_path.segments[3].path.turn.center).angle();
            _next_path.segments[3].path.turn.end_angle =  (tangent4 - _next_path.segments[3].path.turn.center).angle();

            break;
        }

        case TURN_TYPE::LATE: {
            // late turn is two turns, keeping on the first path until set turn point
            _next_path.num_segments = 3;

            // must be able to re-join path with only two turns, do not travel past WP B too far
            _dest_turn_point = MIN(_dest_turn_point, turn_radius * 1.5);

            // turn at given distance befor/after point B
            const Vector2F turn_point = BA_norm * _dest_turn_point;

            // straight segment ends as that turn point
            _next_path.segments[0].type = DUBINS_PATH::SEGMENT::STRAIGHT;
            _next_path.segments[0].path.straight.start = BA;
            _next_path.segments[0].path.straight.vec = -BA_norm - turn_point;

            // left or right turn first?
            Vector2F BA_normal {BA.y, -BA.x};
            if (BA_normal.dot(BC) < 0.0) {
                BA_normal *= -1.0;
                _next_path.segments[1].type = DUBINS_PATH::SEGMENT::RIGHT;
                _next_path.segments[2].type = DUBINS_PATH::SEGMENT::LEFT;
            } else {
                _next_path.segments[1].type = DUBINS_PATH::SEGMENT::LEFT;
                _next_path.segments[2].type = DUBINS_PATH::SEGMENT::RIGHT;
            }

            // first center is tangent to line at turn point
            _next_path.segments[1].path.turn.center = turn_point + BA_normal.normalized() * turn_radius;

            // temporary point is projection of first center onto BC line
            const Vector2F temp_point = BC_norm * _next_path.segments[1].path.turn.center.dot(BC_norm);

            Vector2F BC_normal {BC.y, - BC.x};
            if (BC_normal.dot(BA) > 0.0) {
                BC_normal *= -1.0;
            }

            float perp_dist;
            if (_next_path.segments[1].path.turn.center.dot(BC_norm) > 0.0) {
                perp_dist = turn_radius - (temp_point - _next_path.segments[1].path.turn.center).length();
            } else {
                perp_dist = turn_radius + (temp_point - _next_path.segments[1].path.turn.center).length();
            }

            _next_path.segments[2].path.turn.center = temp_point + (BC_norm * sqrtf(sq(2.0*turn_radius) - sq(perp_dist))) + (BC_normal * turn_radius);

            // tangent is midway between centers
            const Vector2F tangent1 = _next_path.segments[1].path.turn.center + (_next_path.segments[2].path.turn.center - _next_path.segments[1].path.turn.center) * 0.5;

            // tangent is projection of center on to BC line
            const Vector2F tangent2 = BC_norm * _next_path.segments[2].path.turn.center.dot(BC_norm);

            // Calculate start and end angles
            _next_path.segments[1].path.turn.start_angle = (turn_point - _next_path.segments[1].path.turn.center).angle();
            _next_path.segments[1].path.turn.end_angle = (tangent1 - _next_path.segments[1].path.turn.center).angle();
            _next_path.segments[2].path.turn.start_angle = (tangent1 - _next_path.segments[2].path.turn.center).angle();
            _next_path.segments[2].path.turn.end_angle =  (tangent2 - _next_path.segments[2].path.turn.center).angle();

            break;
        }
    }

}

void AP_Navigation::update_waypoint(const struct Location &prev_WP, const struct Location &next_WP)
{
    _prev_dest = prev_WP;
    _dest = next_WP;
    _next_dest.zero();

    _update_waypoint(prev_WP, next_WP, 0);
}
