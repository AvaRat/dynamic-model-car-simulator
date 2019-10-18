#include<iostream>
#include<fstream>
#include<vector>
#include<utility>
#include<cassert>
#include<cmath>
#include<algorithm>

using namespace std;
typedef vector<pair<double, double>> points;

class Path {
    private:
    points pathpoints;
    const size_t points_horizon = 2;

    public:
    Path() {}
    void read_from_file(string path_to_file)
    {
        double x, y;
        fstream f;
        f.open(path_to_file, fstream::in);
        if(!f)
            cout << "error while opening file\n";
        while(f >> x >> y)
        {
            pathpoints.push_back(make_pair(x, y));
        }
        cout << pathpoints.size() << " points read\n";
        f.close();
    }

    points get_n_closest(double x_pos, double y_pos, size_t n_points=2)
    {
        assert(pathpoints.size() > 0);
        vector<double> distances;
        size_t i=0;
        for(auto it=pathpoints.begin(); it!=pathpoints.end(); it++, i++)
        {
            // sqrt((x-x0)^2 + (y-y0)^2)
            distances.push_back(sqrt(pow((*it).first - x_pos, 2) + pow((*it).second - y_pos, 2)));
        }
        auto min_iterator = min_element(distances.begin(), distances.end());
        size_t nth_elem = distance(distances.begin(), min_iterator);
        points closests;
        for(size_t i=0; i<n_points; i++, nth_elem++)
            {
                if(nth_elem == pathpoints.size())
                    nth_elem = 0;
                closests.push_back(pathpoints[nth_elem]);
            }
        assert(closests.size() == n_points);
        return closests;
        //TODO get closest n points on our path
    }

    double get_starting_yaw()
    {
        assert(pathpoints.size() > 0);
        return (get_offsets(pathpoints[0].first, pathpoints[0].second, 0)).second;
    }
/**
 * @retval pair<linear_offset, angular_offset>
 * 
 */
    pair<double, double> get_offsets(double x_pos, double y_pos, double yaw)
    {
        double x1, y1, x2, y2, path_yaw, angular_offset, linear_offset;
        vector<double> coefficients = {};
        points closest_points = get_n_closest(x_pos, y_pos);

        assert(closest_points.size() >= 2);

        x1 = closest_points[0].first;
        y1 = closest_points[0].second;

        x2 = closest_points[1].first;
        y2 = closest_points[1].second;
        //ROS_INFO("%lf %lf %lf %lf", x1, y1, x2, y2);
        //Prosta w okreslona rownaniem ax + by + c = 0
        //a
        coefficients[0] = y1 - y2;
        //b
        coefficients[1] = x2 - x1;
        //c - podstawiamy dowolny punkt, uzyskujemy wyraz wolny i przenosimy go
        //na lewą stronę z minusem
        coefficients[2] = -coefficients[0]*x1 - coefficients[1]*y1;\
        //nachylenie sciezki do osi x
        path_yaw = std::atan2((y2 - y1),(x2 - x1));
        angular_offset = path_yaw - yaw;

        double cross_product = (x_pos - x1) * (y2 - y1) -  (x2 - x1) * (y_pos - y1);
        cout << "cross product " << cross_product << endl;
        int sign;
        if(cross_product < 0) sign = -1;
        else sign = 1;

        linear_offset = sign * std::abs(coefficients[0]*x_pos + coefficients[1]*y_pos +
                         coefficients[2])/std::sqrt(std::pow(coefficients[0],2) + std::pow(coefficients[1],2));
        return make_pair(linear_offset, angular_offset);
    }
};