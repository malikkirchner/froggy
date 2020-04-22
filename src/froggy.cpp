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
#include <iostream>


namespace {

// Gravitational constant
constexpr double G                   = 6.6743015e-11;    // m^3/(kg*s^2)
constexpr double earth_moon_distance = 384'400'000.;     // meters

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

struct ViewPort {
    Eigen::Vector2d origin       = { 500, 100 };
    Eigen::Vector2d dimensions   = { 1800, 1250 };
    double          zoom         = 1200 / earth_moon_distance;
    std::uint16_t   frame_stride = 100;
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
};

cv::Scalar lerp( const double s, const cv::Scalar& a, const cv::Scalar& b ) {
    return { a[ 0 ] + s * ( b[ 0 ] - a[ 0 ] ), a[ 1 ] + s * ( b[ 1 ] - a[ 1 ] ), a[ 2 ] + s * ( b[ 2 ] - a[ 2 ] ) };
}

void draw( cv::Mat& buffer, const Coordinates& center, const double radius, const cv::Scalar& color,
           const ViewPort& view_port, const int thickness = cv::FILLED, const cv::LineTypes line_type = cv::LINE_8 ) {
    const int x = view_port.origin.x() + std::round( center.x.x() * view_port.zoom );
    const int y = view_port.origin.y() + std::round( center.x.y() * view_port.zoom );
    const int r = std::max< int >( std::roundl( radius * view_port.zoom ), 3 );
    cv::circle( buffer, cv::Point( x, y ), r, color, thickness, line_type, 0 );
}

void draw( cv::Mat& buffer, const Body& body, const cv::Scalar& color, const ViewPort& view_port,
           const int thickness = cv::FILLED, const cv::LineTypes line_type = cv::LINE_8 ) {
    draw( buffer, body.coordinates, body.radius, color, view_port, thickness, line_type );
}

void draw( cv::Mat& buffer, const std::vector< Coordinates >& trajectory, const cv::Scalar& color,
           const ViewPort& view_port, const cv::LineTypes line_type = cv::LINE_AA ) {
    const auto stride = view_port.frame_stride;

    for ( unsigned k = stride; k < trajectory.size(); k += stride ) {
        const int x0 = view_port.origin.x() + std::round( trajectory[ k - stride ].x.x() * view_port.zoom );
        const int y0 = view_port.origin.y() + std::round( trajectory[ k - stride ].x.y() * view_port.zoom );
        const int x1 = view_port.origin.x() + std::round( trajectory[ k ].x.x() * view_port.zoom );
        const int y1 = view_port.origin.y() + std::round( trajectory[ k ].x.y() * view_port.zoom );
        cv::line( buffer, cv::Point2d( x0, y0 ), cv::Point2d( x1, y1 ), color, 1, line_type );
    }
}

void draw( cv::Mat& buffer, const std::vector< Coordinates >& trajectory, const double v_lo, const double v_hi,
           const cv::Scalar& color_lo, const cv::Scalar& color_hi, const ViewPort& view_port,
           const cv::LineTypes line_type = cv::LINE_AA ) {
    const auto stride = view_port.frame_stride;

    for ( unsigned k = stride; k < trajectory.size(); k += stride ) {
        const int x0 = view_port.origin.x() + std::round( trajectory[ k - stride ].x.x() * view_port.zoom );
        const int y0 = view_port.origin.y() + std::round( trajectory[ k - stride ].x.y() * view_port.zoom );
        const int x1 = view_port.origin.x() + std::round( trajectory[ k ].x.x() * view_port.zoom );
        const int y1 = view_port.origin.y() + std::round( trajectory[ k ].x.y() * view_port.zoom );

        const double v     = 0.5 * ( trajectory[ k ].v + trajectory[ k - stride ].v ).norm();
        const auto   color = lerp( ( v - v_lo ) / ( v_hi - v_lo ), color_lo, color_hi );
        cv::line( buffer, cv::Point2d( x0, y0 ), cv::Point2d( x1, y1 ), color, 1, line_type );
    }
}

template < std::size_t N >
std::array< Eigen::Vector2d, N > gravitational_acceleration( const std::array< Body, N >& bodies ) {
    std::array< Eigen::Vector2d, N > result;

    for ( unsigned i = 0; i < bodies.size(); ++i ) {
        result[ i ] = { 0., 0. };
        for ( unsigned k = 0; k < bodies.size(); ++k ) {
            if ( i == k ) {
                continue;
            }
            const auto r = bodies[ k ].coordinates.x - bodies[ i ].coordinates.x;
            result[ i ] += G * r.normalized() * bodies[ k ].mass / r.squaredNorm();
        }
    }

    return result;
}

Eigen::Vector2d thrust( const double initial_rocket_mass, const std::vector< Stage >& stages, const Body& rocket,
                        const Body& moon, const double t ) {
    double k = 0.;
    double m = initial_rocket_mass;
    for ( const auto& stage : stages ) {
        const auto a = m - stage.mass / stage.duration * ( t - k );
        k += stage.duration;
        m -= stage.mass;
        if ( t < k ) {
            const Eigen::Rotation2Dd rot{ M_PI / 180. * 20.3 };
            const auto               r = moon.coordinates.x - rocket.coordinates.x;
            return stage.thrust / a * ( rot * r.normalized() );
        }
    }

    return { 0., 0. };
}


template < std::size_t N >
void leap_frog( std::array< Body, N >& bodies, const double initial_rocket_mass, const std::vector< Stage >& stages,
                const double t, const double dt ) {
    std::array< Body, N > half_step = bodies;

    auto g = gravitational_acceleration( bodies );

    Eigen::Vector2d acc;
    for ( unsigned k = 0; k < N; ++k ) {
        acc = g[ k ] + ( ( k == 0 ) ? 12.803 * thrust( initial_rocket_mass, stages, bodies[ 0 ], bodies[ 2 ], t )
                                    : Eigen::Vector2d{ 0., 0. } );
        half_step[ k ].coordinates.v = bodies[ k ].coordinates.v + acc * 0.5 * dt;
        half_step[ k ].coordinates.x = bodies[ k ].coordinates.x + half_step[ k ].coordinates.v * 0.5 * dt;
    }

    g = gravitational_acceleration( half_step );

    for ( unsigned k = 0; k < N; ++k ) {
        bodies[ k ].coordinates.x = half_step[ k ].coordinates.x + half_step[ k ].coordinates.v * 0.5 * dt;
        acc = g[ k ] + ( ( k == 0 ) ? 12.803 * thrust( initial_rocket_mass, stages, bodies[ 0 ], bodies[ 2 ], t )
                                    : Eigen::Vector2d{ 0., 0. } );
        bodies[ k ].coordinates.v = half_step[ k ].coordinates.v + acc * 0.5 * dt;
    }
}

}    // namespace


class Scene {
public:
    Scene() {
        earth.mass    = 5.9722e24;
        earth.radius  = 6'371'000.;
        moon.mass     = 7.34767309e22;
        moon.radius   = 1'737'100.;
        rocket.mass   = 750'000;
        rocket.radius = 27.;

        const double M      = earth.mass + moon.mass;
        earth.coordinates.x = { -earth_moon_distance * moon.mass / M, 0. };
        moon.coordinates.x  = { earth_moon_distance * earth.mass / M, 0. };
        earth.coordinates.v = { 0., -2. * M_PI * std::abs( earth.coordinates.x.x() ) / ( 29 * 24 * 3600. ) };
        moon.coordinates.v  = { 0., +2. * M_PI * std::abs( moon.coordinates.x.x() ) / ( 29 * 24 * 3600. ) };

        rocket.coordinates.x = { earth.coordinates.x.x() + earth.radius, earth.coordinates.x.y() };
        rocket.coordinates.v = { earth.coordinates.v.x(),
                                 earth.coordinates.v.y() + 2. * M_PI * cos( M_PI * ( 23.4 + 5.14 ) / 180. ) *
                                                                   earth.radius / ( 24. * 3600. ) };

        Stage booster_stage;
        booster_stage.thrust   = 4'000'000;
        booster_stage.duration = 130.;
        booster_stage.mass     = 270'000;

        Stage main_stage;
        main_stage.thrust   = 1'180'000;
        main_stage.duration = 605.;
        main_stage.mass     = 170'500;

        Stage upper_stage;
        main_stage.thrust   = 27'000;
        main_stage.duration = 1100.;
        main_stage.mass     = 10'900;

        stages.push_back( std::move( booster_stage ) );
        stages.push_back( std::move( main_stage ) );
        stages.push_back( std::move( upper_stage ) );

        initial_rocket_mass = rocket.mass;

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
    }

    bool integrate() {
        std::array< Body, 3 > bodies{ rocket, earth, moon };
        leap_frog( bodies, initial_rocket_mass, stages, t, dt );
        rocket = bodies[ 0 ];
        earth  = bodies[ 1 ];
        moon   = bodies[ 2 ];
        t += dt;
        ++k;

        if ( k % view_port.frame_stride == 0 ) {
            rocket_trajectory.emplace_back( Coordinates{ t, rocket.coordinates.x, rocket.coordinates.v } );
            earth_trajectory.emplace_back( Coordinates{ t, earth.coordinates.x, earth.coordinates.v } );
            moon_trajectory.emplace_back( Coordinates{ t, moon.coordinates.x, moon.coordinates.v } );
        }

        return ( rocket.coordinates.x - earth.coordinates.x ).norm() >= earth.radius;
    }

public:
    ViewPort view_port{};

    double               initial_rocket_mass = 0.;
    Body                 rocket;
    Body                 earth;
    Body                 moon;
    std::vector< Stage > stages;

    double        t  = 0.;
    double        T  = 10 * 24 * 3600;
    double        dt = 0.01;
    std::uint64_t k  = 0;

    std::vector< Coordinates > rocket_trajectory;
    std::vector< Coordinates > earth_trajectory;
    std::vector< Coordinates > moon_trajectory;

    Stats stats;
};


int main() {
#ifdef USE_GTK
    cv::namedWindow( "froggy", cv::WINDOW_AUTOSIZE );
    cv::moveWindow( "froggy", 0, 0 );
#endif

    Scene scene;

    for ( scene.t = 0; scene.t < scene.T; ) {
        scene.update_stats();

        if ( !scene.integrate() ) {
            break;
        };
    }

    const cv::Mat      buffer = scene.render();
    std::vector< int > compression_params;
    compression_params.push_back( cv::IMWRITE_PNG_COMPRESSION );
    compression_params.push_back( 6 );
    cv::imwrite( "moon_shot.png", buffer, compression_params );

#ifdef USE_GTK
    cv::imshow( "froggy", buffer );
    cv::waitKey( 0 );
#endif

    return 0;
}