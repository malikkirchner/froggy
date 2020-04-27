/*====================================================================================================================*/
/*                                                                                                                    */
/*      FROGGY     Copyright ©2020 by  Malik Kirchner <kirchner@xelonic.com>                                          */
/*                                                                                                                    */
/*      This program is free software: you can redistribute it and/or modify                                          */
/*      it under the terms of the GNU General Public License as published by                                          */
/*      the Free Software Foundation, either version 3 of the License, or                                             */
/*      (at your option) any later version.                                                                           */
/*                                                                                                                    */
/*      This program is distributed in the hope that it will be useful,                                               */
/*      but WITHOUT ANY WARRANTY; without even the implied warranty of                                                */
/*      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                                                 */
/*      GNU General Public License for more details.                                                                  */
/*                                                                                                                    */
/*      You should have received a copy of the GNU General Public License                                             */
/*      along with this program.  If not, see <https://www.gnu.org/licenses/>.                                        */
/*                                                                                                                    */
/*====================================================================================================================*/

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#ifdef USE_GTK
#include <opencv2/highgui.hpp>
#endif

#include <Eigen/Dense>
#include <spdlog/spdlog.h>

#include <array>
#include <cmath>
#include <filesystem>
#include <iostream>


namespace {

constexpr double rad( const double deg ) noexcept { return deg * M_PI / 180.; }
constexpr double circumference( const double r ) noexcept { return 2.0 * M_PI * std::abs( r ); }

// Gravitational constant
constexpr double G                   = 6.6743015e-11;           // m^3/(kg*s^2)
constexpr double earth_moon_distance = 384'400'000.;            // meters
constexpr double moon_period         = 27.322 * 24. * 3600.;    // seconds
constexpr double tilt_earth_axis     = rad( 23.4 );             // radian
constexpr double tilt_lunar_plane    = rad( 5.14 );             // radian

const cv::Scalar color_rocket{ 0, 100, 255 };
const cv::Scalar color_rocket_trajectory_lo{ 0, 50, 255 };
const cv::Scalar color_rocket_trajectory_hi{ 0, 255, 255 };
const cv::Scalar color_earth{ 255, 50, 50 };
const cv::Scalar color_earth_trajectory{ 200, 200, 200 };
const cv::Scalar color_moon{ 200, 200, 200 };
const cv::Scalar color_moon_trajectory{ 200, 200, 200 };


struct Coordinates {
    double          t = 0.;
    Eigen::Vector2d x = { 0., 0. };    // position, in meters from origin
    Eigen::Vector2d v = { 0., 0. };    // velocity, in meters per second
    Eigen::Vector2d a = { 0., 0. };    // acceleration, in meters per second squared
};

struct Body {
    Coordinates coordinates{};    // meter from origin
    double      mass   = 0.;      // kilogram
    double      radius = 0.;      // meter
};

struct Stage {
    double thrust   = 0.;    // Newton
    double duration = 0.;    // seconds
    double mass     = 0.;    // kilogram
};

class ViewPort {
public:
    Eigen::Vector2i origin       = { 0, 0 };
    Eigen::Vector2i dimensions   = { 0, 0 };
    double          zoom         = 1.;
    std::uint16_t   frame_stride = 100;

    void adapt() {
        const auto s      = max_x - min_x;
        const auto zoom_x = max_width / ( 1.1 * s.x() );
        const auto zoom_y = max_height / ( 1.1 * s.y() );

        if ( zoom_x < zoom_y ) {
            zoom       = zoom_x;
            dimensions = { max_width, 1.1 * s.y() * zoom };
        } else {
            zoom       = zoom_y;
            dimensions = { 1.1 * s.x() * zoom, max_height };
        }

        origin = { -std::roundl( ( -0.05 * s.x() + min_x.x() ) * zoom ),
                   -std::roundl( ( -0.05 * s.y() + min_x.y() ) * zoom ) };
    }

    void adapt( const Body& body ) {
        min_x.x() = std::min( min_x.x(), body.coordinates.x.x() - body.radius );
        max_x.x() = std::max( max_x.x(), body.coordinates.x.x() + body.radius );
        min_x.y() = std::min( min_x.y(), body.coordinates.x.y() - body.radius );
        max_x.y() = std::max( max_x.y(), body.coordinates.x.y() + body.radius );
        adapt();
    }

private:
    double          max_width  = 1920;
    double          max_height = 1080;
    Eigen::Vector2d min_x      = { std::numeric_limits< double >::max(), std::numeric_limits< double >::max() };
    Eigen::Vector2d max_x      = { std::numeric_limits< double >::min(), std::numeric_limits< double >::min() };
};


struct Stats {
    std::uint64_t closest_rocket_moon_index      = 0;
    std::uint64_t farthest_rocket_earth_index    = 0;
    double        closest_rocket_moon_distance   = std::numeric_limits< double >::max();
    double        farthest_rocket_earth_distance = 0.;

    std::uint64_t lowest_velocity_index  = 0;
    std::uint64_t highest_velocity_index = 0;
    double        lowest_velocity        = std::numeric_limits< double >::max();
    double        highest_velocity       = 0.;

    std::uint64_t lowest_acceleration_index  = 0;
    std::uint64_t highest_acceleration_index = 0;
    double        lowest_acceleration        = std::numeric_limits< double >::max();
    double        highest_acceleration       = 0.;

    double rocket_orbit_length = 0.;
    double earth_orbit_length  = 0.;
    double moon_orbit_length   = 0.;

    std::chrono::system_clock::time_point simulation_start = std::chrono::system_clock::now();
};

cv::Scalar lerp( const double s, const cv::Scalar& a, const cv::Scalar& b ) {
    return { a[ 0 ] + s * ( b[ 0 ] - a[ 0 ] ), a[ 1 ] + s * ( b[ 1 ] - a[ 1 ] ), a[ 2 ] + s * ( b[ 2 ] - a[ 2 ] ) };
}

void draw( cv::Mat& buffer, const Coordinates& center, const double radius, const cv::Scalar& color,
           const ViewPort& view_port, const int thickness = cv::FILLED, const cv::LineTypes line_type = cv::LINE_8 ) {
    const int x = view_port.origin.x() + std::roundl( center.x.x() * view_port.zoom );
    const int y = view_port.origin.y() + std::roundl( center.x.y() * view_port.zoom );
    const int r = std::max< int >( std::roundl( radius * view_port.zoom ), 3 );
    cv::circle( buffer, cv::Point2i( x, y ), r, color, thickness, line_type, 0 );
}

void draw( cv::Mat& buffer, const Body& body, const cv::Scalar& color, const ViewPort& view_port,
           const int thickness = cv::FILLED, const cv::LineTypes line_type = cv::LINE_8 ) {
    draw( buffer, body.coordinates, body.radius, color, view_port, thickness, line_type );
}

void draw( cv::Mat& buffer, const std::vector< Coordinates >& trajectory, const cv::Scalar& color,
           const ViewPort& view_port, const cv::LineTypes line_type = cv::LINE_AA ) {
    for ( unsigned k = 1; k < trajectory.size(); ++k ) {
        const int x0 = view_port.origin.x() + std::roundl( trajectory[ k - 1 ].x.x() * view_port.zoom );
        const int y0 = view_port.origin.y() + std::roundl( trajectory[ k - 1 ].x.y() * view_port.zoom );
        const int x1 = view_port.origin.x() + std::roundl( trajectory[ k ].x.x() * view_port.zoom );
        const int y1 = view_port.origin.y() + std::roundl( trajectory[ k ].x.y() * view_port.zoom );
        cv::line( buffer, cv::Point2i( x0, y0 ), cv::Point2i( x1, y1 ), color, 1, line_type );
    }
}

void draw( cv::Mat& buffer, const std::vector< Coordinates >& trajectory, const double v_lo, const double v_hi,
           const cv::Scalar& color_lo, const cv::Scalar& color_hi, const ViewPort& view_port,
           const cv::LineTypes line_type = cv::LINE_AA ) {
    for ( unsigned k = 1; k < trajectory.size(); ++k ) {
        const int x0 = view_port.origin.x() + std::round( trajectory[ k - 1 ].x.x() * view_port.zoom );
        const int y0 = view_port.origin.y() + std::round( trajectory[ k - 1 ].x.y() * view_port.zoom );
        const int x1 = view_port.origin.x() + std::round( trajectory[ k ].x.x() * view_port.zoom );
        const int y1 = view_port.origin.y() + std::round( trajectory[ k ].x.y() * view_port.zoom );

        const double v     = 0.5 * ( trajectory[ k ].v + trajectory[ k - 1 ].v ).norm();
        const auto   color = lerp( ( v - v_lo ) / ( v_hi - v_lo ), color_lo, color_hi );
        cv::line( buffer, cv::Point2i( x0, y0 ), cv::Point2i( x1, y1 ), color, 1, line_type );
    }
}

template < std::size_t N >
std::array< Eigen::Vector2d, N > gravitational_acceleration( const std::array< Body, N >& bodies ) {
    std::array< Eigen::Vector2d, N > result;

    for ( unsigned i = 0; i < N; ++i ) {
        result[ i ] = { 0., 0. };
        for ( unsigned k = 0; k < i; ++k ) {
            const auto x = bodies[ k ].coordinates.x - bodies[ i ].coordinates.x;
            const auto l = x.norm();
            const auto n = G * x / ( l * l * l );

            result[ i ] += n * bodies[ k ].mass;
            result[ k ] -= n * bodies[ i ].mass;
        }
    }

    return result;
}

Eigen::Vector2d thrust( const double initial_rocket_mass, const double rocket_launch_tilt,
                        const std::vector< Stage >& stages, const Body& rocket, const Body& moon, const double t ) {
    double k = 0.;
    double m = initial_rocket_mass;
    for ( const auto& stage : stages ) {
        k += stage.duration;
        m -= stage.mass;
        if ( t < k ) {
            const auto               a = m - stage.mass / stage.duration * ( t - k );
            const Eigen::Rotation2Dd rot{ rocket_launch_tilt };
            const auto               r = moon.coordinates.x - rocket.coordinates.x;
            return stage.thrust / a * ( rot * r.normalized() );
        }
    }

    return { 0., 0. };
}


template < std::size_t N >
void leap_frog( std::array< Body, N >& bodies, const double initial_rocket_mass, const double rocket_launch_tilt,
                const double thrust_shutdown_time, const std::vector< Stage >& stages, const double t,
                const double dt ) {
    std::array< Body, N > half_step = bodies;

    auto g = gravitational_acceleration( bodies );

    Eigen::Vector2d acc;
    for ( unsigned k = 0; k < N; ++k ) {
        acc = g[ k ] + ( ( k == 0 && t < thrust_shutdown_time ) ? thrust( initial_rocket_mass, rocket_launch_tilt,
                                                                          stages, bodies[ 0 ], bodies[ 2 ], t )
                                                                : Eigen::Vector2d{ 0., 0. } );
        half_step[ k ].coordinates.v = bodies[ k ].coordinates.v + acc * 0.5 * dt;
        half_step[ k ].coordinates.x = bodies[ k ].coordinates.x + half_step[ k ].coordinates.v * 0.5 * dt;
    }

    g = gravitational_acceleration( half_step );

    for ( unsigned k = 0; k < N; ++k ) {
        bodies[ k ].coordinates.x = half_step[ k ].coordinates.x + half_step[ k ].coordinates.v * 0.5 * dt;
        acc = g[ k ] + ( ( k == 0 && t < thrust_shutdown_time ) ? thrust( initial_rocket_mass, rocket_launch_tilt,
                                                                          stages, bodies[ 0 ], bodies[ 2 ], t )
                                                                : Eigen::Vector2d{ 0., 0. } );
        bodies[ k ].coordinates.v = half_step[ k ].coordinates.v + acc * 0.5 * dt;
        bodies[ k ].coordinates.a = acc;
    }
}

}    // namespace


class Scene {
private:
    ViewPort view_port{};

    double initial_rocket_mass  = 0.;    // kilogram
    double rocket_launch_tilt   = 0.;    // radians
    double thrust_shutdown_time = 0.;    // seconds

    Body                 rocket;
    Body                 earth;
    Body                 moon;
    std::vector< Stage > stages;

    double        t  = 0.;
    double        T  = moon_period;
    double        dt = 0.01;
    std::uint64_t k  = 0;

    std::vector< Coordinates > rocket_trajectory;
    std::vector< Coordinates > earth_trajectory;
    std::vector< Coordinates > moon_trajectory;

    Stats stats;

public:
    Scene() {
        earth.mass    = 5.9722e24;
        earth.radius  = 6'371'000.;
        moon.mass     = 7.34767309e22;
        moon.radius   = 1'737'100.;
        rocket.mass   = 750'000;
        rocket.radius = 55.;

        const double M      = earth.mass + moon.mass;
        earth.coordinates.x = { -earth_moon_distance * moon.mass / M, 0. };
        moon.coordinates.x  = { earth_moon_distance * earth.mass / M, 0. };
        earth.coordinates.v = { 0., -circumference( earth.coordinates.x.x() ) / moon_period };
        moon.coordinates.v  = { 0., +circumference( moon.coordinates.x.x() ) / moon_period };

        rocket.coordinates.x = { earth.coordinates.x.x() + earth.radius, earth.coordinates.x.y() };
        rocket.coordinates.v = { earth.coordinates.v.x(),
                                 earth.coordinates.v.y() +
                                         circumference( cos( tilt_earth_axis + tilt_lunar_plane ) * earth.radius ) /
                                                 ( 24. * 3600. ) };

        const double ariane_boost = 5.00585;

        Stage booster_stage;
        booster_stage.thrust   = ariane_boost * 4'000'000;
        booster_stage.duration = 130.;
        booster_stage.mass     = 270'000;
        thrust_shutdown_time += booster_stage.duration;

        Stage main_stage;
        main_stage.thrust   = ariane_boost * 1'180'000;
        main_stage.duration = 605.;
        main_stage.mass     = 170'500;
        thrust_shutdown_time += main_stage.duration;

        Stage upper_stage;
        upper_stage.thrust   = ariane_boost * 27'000;
        upper_stage.duration = 1100.;
        upper_stage.mass     = 10'900;
        thrust_shutdown_time += upper_stage.duration;

        stages.push_back( std::move( booster_stage ) );
        stages.push_back( std::move( main_stage ) );
        stages.push_back( std::move( upper_stage ) );

        initial_rocket_mass = rocket.mass;
        rocket_launch_tilt  = rad( 23.0 );

        rocket_trajectory.reserve( T / dt / view_port.frame_stride + 100 );
        earth_trajectory.reserve( T / dt / view_port.frame_stride + 100 );
        moon_trajectory.reserve( T / dt / view_port.frame_stride + 100 );
        rocket_trajectory.emplace_back( Coordinates{ t, rocket.coordinates.x, rocket.coordinates.v } );
        earth_trajectory.emplace_back( Coordinates{ t, earth.coordinates.x, earth.coordinates.v } );
        moon_trajectory.emplace_back( Coordinates{ t, moon.coordinates.x, moon.coordinates.v } );
    }

    cv::Mat render() const {
        cv::Mat buffer( view_port.dimensions.y(), view_port.dimensions.x(), CV_8UC3 );

        draw( buffer, earth, color_earth, view_port );
        draw( buffer, moon, color_moon, view_port );
        draw( buffer, rocket, color_rocket, view_port );

        draw( buffer, earth_trajectory[ 0 ], earth.radius, color_earth_trajectory, view_port, 1, cv::LINE_AA );
        draw( buffer, moon_trajectory[ 0 ], moon.radius, color_moon_trajectory, view_port, 1, cv::LINE_AA );

        draw( buffer, rocket_trajectory[ 0 ], rocket.radius, color_rocket, view_port, 1, cv::LINE_AA );

        draw( buffer, moon_trajectory[ stats.closest_rocket_moon_index / view_port.frame_stride ], moon.radius,
              color_moon, view_port, 1, cv::LINE_AA );

        draw( buffer, earth_trajectory, color_earth_trajectory, view_port );
        draw( buffer, moon_trajectory, color_moon_trajectory, view_port );

        draw( buffer, rocket_trajectory, stats.lowest_velocity, stats.highest_velocity, color_rocket_trajectory_lo,
              color_rocket_trajectory_hi, view_port );

        return buffer;
    }


    void update_stats() {
        const double rocket_earth_distance = ( rocket.coordinates.x - earth.coordinates.x ).norm();
        const double rocket_moon_distance  = ( rocket.coordinates.x - moon.coordinates.x ).norm();

        if ( stats.closest_rocket_moon_distance > rocket_moon_distance ) {
            stats.closest_rocket_moon_distance = rocket_moon_distance;
            stats.closest_rocket_moon_index    = k;
        }

        if ( stats.farthest_rocket_earth_distance < rocket_earth_distance ) {
            stats.farthest_rocket_earth_distance = rocket_earth_distance;
            stats.farthest_rocket_earth_index    = k;
        }

        const double v = rocket.coordinates.v.norm();
        if ( stats.lowest_velocity > v ) {
            stats.lowest_velocity       = v;
            stats.lowest_velocity_index = k;
        }
        if ( stats.highest_velocity < v ) {
            stats.highest_velocity       = v;
            stats.highest_velocity_index = k;
        }

        const double a = rocket.coordinates.a.norm();
        if ( stats.lowest_acceleration > a ) {
            stats.lowest_acceleration       = a;
            stats.lowest_acceleration_index = k;
        }
        if ( stats.highest_acceleration < a ) {
            stats.highest_acceleration       = a;
            stats.highest_acceleration_index = k;
        }
    }

    bool integrate() {
        std::array< Body, 3 > bodies{ rocket, earth, moon };
        leap_frog( bodies, initial_rocket_mass, rocket_launch_tilt, thrust_shutdown_time, stages, t, dt );
        update_trajectory_lengths( bodies );
        rocket = bodies[ 0 ];
        earth  = bodies[ 1 ];
        moon   = bodies[ 2 ];
        t += dt;
        ++k;

        const bool done = ( t >= T ) || ( rocket.coordinates.x - earth.coordinates.x ).norm() < earth.radius;
        if ( done || k % view_port.frame_stride == 0 ) {
            record_trajectories();
            adapt_view_port();
        }
        return !done;
    }

    std::string str() {
        std::stringstream s;

        double duration = 0.001 * std::chrono::duration_cast< std::chrono::milliseconds >(
                                          std::chrono::system_clock::now() - stats.simulation_start )
                                          .count();

        s << fmt::format( "Iterations               : {} ({:1.3f}/s)\n", k, k / duration );
        s << fmt::format( "Simulation duration      : {:1.3f}s\n", duration );

        s << fmt::format( "Flight duration          : {:1.2f}s ({:1.2f}d)\n", t, t / 24. / 3600. );

        s << fmt::format( "Lowest velocity          : {:1.2f}m/s ({:1.2f}km/h)\n", stats.lowest_velocity,
                          stats.lowest_velocity * 3.6 );
        s << fmt::format( "Hightest velocity        : {:1.2f}m/s ({:1.2f}km/h)\n", stats.highest_velocity,
                          stats.highest_velocity * 3.6 );
        s << fmt::format( "Lowest acceleration      : {:1.2f}m/s^2 ({:1.2f}g)\n", stats.lowest_acceleration,
                          stats.lowest_acceleration / 9.81 );
        s << fmt::format( "Hightest acceleration    : {:1.2f}m/s^2 ({:1.2f}g)\n", stats.highest_acceleration,
                          stats.highest_acceleration / 9.81 );

        s << fmt::format( "Rocket trajectory length : {:1.3f}km\n", 0.001 * stats.rocket_orbit_length );
        s << fmt::format( "Earth trajectory length  : {:1.3f}km\n", 0.001 * stats.earth_orbit_length );
        s << fmt::format( "Moon trajectory length   : {:1.3f}km\n", 0.001 * stats.moon_orbit_length );

        return s.str();
    }

private:
    void update_trajectory_lengths( const std::array< Body, 3 >& bodies ) {
        stats.rocket_orbit_length += ( bodies[ 0 ].coordinates.x - rocket.coordinates.x ).norm();
        stats.earth_orbit_length += ( bodies[ 1 ].coordinates.x - earth.coordinates.x ).norm();
        stats.moon_orbit_length += ( bodies[ 2 ].coordinates.x - moon.coordinates.x ).norm();
    }

    void record_trajectories() {
        rocket_trajectory.emplace_back( Coordinates{ t, rocket.coordinates.x, rocket.coordinates.v } );
        earth_trajectory.emplace_back( Coordinates{ t, earth.coordinates.x, earth.coordinates.v } );
        moon_trajectory.emplace_back( Coordinates{ t, moon.coordinates.x, moon.coordinates.v } );
    }

    void adapt_view_port() {
        view_port.adapt( rocket );
        view_port.adapt( earth );
        view_port.adapt( moon );
    }
};


int main() {
#ifdef USE_GTK
    cv::namedWindow( "froggy", cv::WINDOW_AUTOSIZE );
    cv::moveWindow( "froggy", 0, 0 );
#endif

    spdlog::info( "Starting simulation ..." );
    Scene scene;

    while ( scene.integrate() ) {
        scene.update_stats();
    }

    const auto filename = std::filesystem::current_path().append( "moon_shot.png" ).string();
    spdlog::info( "Rendering orbits ..." );
    spdlog::info( "Please find the diagram at {}", filename );
    const cv::Mat      buffer = scene.render();
    std::vector< int > compression_params;
    compression_params.push_back( cv::IMWRITE_PNG_COMPRESSION );
    compression_params.push_back( 2 );
    cv::imwrite( filename, buffer, compression_params );

    spdlog::info( "Flight stats:\n{}", scene.str() );
#ifdef USE_GTK
    spdlog::info( "Press any key to quit!" );
    cv::imshow( "froggy", buffer );
    cv::waitKey( 0 );
#endif

    return 0;
}