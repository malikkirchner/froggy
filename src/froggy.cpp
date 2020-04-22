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
const cv::Scalar color_rocket_trajectory{ 0, 100, 255 };
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


void draw( cv::Mat& buffer, const Coordinates& center, const double radius, const cv::Scalar& color,
           const ViewPort& view_port ) {
    const int x = view_port.origin.x() + std::round( center.x.x() * view_port.zoom );
    const int y = view_port.origin.y() + std::round( center.x.y() * view_port.zoom );
    const int r = std::max< int >( std::roundl( radius * view_port.zoom ), 1 );
    cv::circle( buffer, cv::Point( x, y ), r, color, cv::FILLED, 8, 0 );
}

void draw( cv::Mat& buffer, const Body& body, const cv::Scalar& color, const ViewPort& view_port ) {
    draw( buffer, body.coordinates, body.radius, color, view_port );
}

void draw( cv::Mat& buffer, const std::vector< Coordinates >& trajectory, const cv::Scalar& color,
           const ViewPort& view_port ) {
    const auto stride = view_port.frame_stride;

    for ( unsigned k = stride; k < trajectory.size(); k += stride ) {
        const int x0 = view_port.origin.x() + std::round( trajectory[ k - stride ].x.x() * view_port.zoom );
        const int y0 = view_port.origin.y() + std::round( trajectory[ k - stride ].x.y() * view_port.zoom );
        const int x1 = view_port.origin.x() + std::round( trajectory[ k ].x.x() * view_port.zoom );
        const int y1 = view_port.origin.y() + std::round( trajectory[ k ].x.y() * view_port.zoom );
        cv::line( buffer, cv::Point2d( x0, y0 ), cv::Point2d( x1, y1 ), color );
    }
}

Eigen::Vector2d gravitational_force( const Body& a, const Body& b, const Body& c ) {
    const auto r_b = b.coordinates.x - a.coordinates.x;
    const auto r_c = c.coordinates.x - a.coordinates.x;

    return G * ( r_b.normalized() * b.mass / r_b.squaredNorm() + r_c.normalized() * c.mass / r_c.squaredNorm() );
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


}    // namespace


struct Scene {
    ViewPort view_port{};

    double               initial_rocket_mass = 0.;
    Body                 rocket;
    Body                 earth;
    Body                 moon;
    std::vector< Stage > stages;

    double        t  = 0.;
    double        dt = 0.01;
    std::uint64_t k  = 0;

    std::vector< Coordinates > rocket_trajectory;
    std::vector< Coordinates > earth_trajectory;
    std::vector< Coordinates > moon_trajectory;

    Stats stats;

    cv::Mat render() const {
        cv::Mat buffer( view_port.dimensions.y(), view_port.dimensions.x(), CV_8UC3 );

        draw( buffer, earth, color_earth, view_port );
        draw( buffer, moon, color_moon, view_port );
        draw( buffer, moon_trajectory[ stats.closest_rocket_moon_index / view_port.frame_stride ], moon.radius,
              color_moon, view_port );
        draw( buffer, rocket, color_rocket, view_port );

        draw( buffer, earth_trajectory, color_earth_trajectory, view_port );
        draw( buffer, moon_trajectory, color_moon_trajectory, view_port );
        draw( buffer, rocket_trajectory, color_rocket_trajectory, view_port );

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
    }

    bool leap_frog() {
        auto aux_rocket = rocket;
        auto aux_earth  = earth;
        auto aux_moon   = moon;

        {
            const auto acc0 = gravitational_force( rocket, earth, moon ) +
                              12.803 * thrust( initial_rocket_mass, stages, rocket, moon, t );

            const auto v = aux_rocket.coordinates.v + acc0 * 0.5 * dt;
            const auto x = aux_rocket.coordinates.x + v * 0.5 * dt;

            aux_rocket.coordinates.x = x + v * 0.5 * dt;
            const auto acc1          = gravitational_force( aux_rocket, earth, moon ) +
                              12.803 * thrust( initial_rocket_mass, stages, aux_rocket, moon, t + 0.5 * dt );
            aux_rocket.coordinates.v = v + acc1 * 0.5 * dt;
        }


        {
            const auto acc0 = gravitational_force( moon, earth, rocket );
            const auto v    = aux_moon.coordinates.v + acc0 * 0.5 * dt;
            const auto x    = aux_moon.coordinates.x + v * 0.5 * dt;

            aux_moon.coordinates.x = x + v * 0.5 * dt;
            const auto acc1        = gravitational_force( aux_moon, earth, rocket );
            aux_moon.coordinates.v = v + acc1 * 0.5 * dt;
        }

        {
            const auto acc0 = gravitational_force( earth, moon, rocket );
            const auto v    = aux_earth.coordinates.v + acc0 * 0.5 * dt;
            const auto x    = aux_earth.coordinates.x + v * 0.5 * dt;

            aux_earth.coordinates.x = x + v * 0.5 * dt;
            const auto acc1         = gravitational_force( aux_earth, moon, rocket );
            aux_earth.coordinates.v = v + acc1 * 0.5 * dt;
        }

        rocket = aux_rocket;
        earth  = aux_earth;
        moon   = aux_moon;
        t += dt;
        ++k;

        if ( k % view_port.frame_stride == 0 ) {
            rocket_trajectory.emplace_back( Coordinates{ t, rocket.coordinates.x, rocket.coordinates.v } );
            earth_trajectory.emplace_back( Coordinates{ t, earth.coordinates.x, earth.coordinates.v } );
            moon_trajectory.emplace_back( Coordinates{ t, moon.coordinates.x, moon.coordinates.v } );
        }

        return ( rocket.coordinates.x - earth.coordinates.x ).norm() >= earth.radius;
    }
};


int main() {
    Scene scene;
    scene.earth.mass    = 5.9722e24;
    scene.earth.radius  = 6'371'000.;
    scene.moon.mass     = 7.34767309e22;
    scene.moon.radius   = 1'737'100.;
    scene.rocket.mass   = 750'000;
    scene.rocket.radius = 27.;

    const double M            = scene.earth.mass + scene.moon.mass;
    scene.earth.coordinates.x = { -earth_moon_distance * scene.moon.mass / M, 0. };
    scene.moon.coordinates.x  = { earth_moon_distance * scene.earth.mass / M, 0. };
    scene.earth.coordinates.v = { 0., -2. * M_PI * std::abs( scene.earth.coordinates.x.x() ) / ( 29 * 24 * 3600. ) };
    scene.moon.coordinates.v  = { 0., +2. * M_PI * std::abs( scene.moon.coordinates.x.x() ) / ( 29 * 24 * 3600. ) };

    scene.rocket.coordinates.x = { scene.earth.coordinates.x.x() + scene.earth.radius, scene.earth.coordinates.x.y() };
    scene.rocket.coordinates.v = { scene.earth.coordinates.v.x(),
                                   scene.earth.coordinates.v.y() + 2. * M_PI * cos( M_PI * ( 23.4 + 5.14 ) / 180. ) *
                                                                           scene.earth.radius / ( 24. * 3600. ) };

    scene.rocket_trajectory.emplace_back(
            Coordinates{ scene.t, scene.rocket.coordinates.x, scene.rocket.coordinates.v } );
    scene.earth_trajectory.emplace_back( Coordinates{ scene.t, scene.earth.coordinates.x, scene.earth.coordinates.v } );
    scene.moon_trajectory.emplace_back( Coordinates{ scene.t, scene.moon.coordinates.x, scene.moon.coordinates.v } );

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

    scene.stages.push_back( std::move( booster_stage ) );
    scene.stages.push_back( std::move( main_stage ) );
    scene.stages.push_back( std::move( upper_stage ) );

    scene.initial_rocket_mass = scene.rocket.mass;

#ifdef USE_GTK
    cv::namedWindow( "froggy", cv::WINDOW_AUTOSIZE );
    cv::moveWindow( "froggy", 0, 0 );
#endif

    const double T = 10 * 24 * 3600;
    scene.rocket_trajectory.reserve( T / scene.dt / scene.view_port.frame_stride + 100 );
    scene.earth_trajectory.reserve( T / scene.dt / scene.view_port.frame_stride + 100 );
    scene.moon_trajectory.reserve( T / scene.dt / scene.view_port.frame_stride + 100 );
    for ( scene.t = 0; scene.t < T; ) {
        scene.update_stats();

        if ( !scene.leap_frog() ) {
            break;
        };
    }

    const cv::Mat buffer = scene.render();

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