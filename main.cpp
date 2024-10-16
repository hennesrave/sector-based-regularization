#include "qapplication.h"
#include "qevent.h"
#include "qlayout.h"
#include "qpainter.h"
#include "qwidget.h"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numbers>
#include <random>
#include <unordered_map>

namespace
{
    std::ostream& operator<<( std::ostream& stream, const QPointF point )
    {
        return stream << '(' << point.x() << ", " << point.y() << ')';
    }
}

struct Sector
{
    struct
    {
        QPointF begin {};
        QPointF center {};
        QPointF end {};
    } intersection;

    struct
    {
        QPointF density {};
        QPointF boundary {};
        QPointF uniform {};
    } deformation {};

    QPointF anchor {};
    double area {};
    double length {};
    double points_count {};
};

struct SquareDomain
{
    static inline const auto bottomleft = QPointF { -1.0, -1.0 };
    static inline const auto bottomright = QPointF { 1.0, -1.0 };
    static inline const auto topleft = QPointF { -1.0, 1.0 };
    static inline const auto topright = QPointF { 1.0, 1.0 };

    static inline const auto left = QLineF { bottomleft, topleft };
    static inline const auto top = QLineF { topleft, topright };
    static inline const auto right = QLineF { bottomright, topright };
    static inline const auto bottom = QLineF { bottomleft, bottomright };

    static inline double total_area()
    {
        return 4.0;
    }
    static inline double total_circumference()
    {
        return 8.0;
    }

    static inline double compute_area( const QPointF& a, const QPointF& b, const QPointF& c )
    {
        return 0.5 * std::abs( a.x() * ( b.y() - c.y() ) + b.x() * ( c.y() - a.y() ) + c.x() * ( a.y() - b.y() ) );
    }

    Sector sector( QPointF position, double radian_begin, double radian_end ) const
    {
        Sector sector {};

        const double radian_center = ( radian_begin + radian_end ) / 2.0;

        const QLineF sector_begin_line { position, position + 10.0 * QPointF { std::cos( radian_begin ), std::sin( radian_begin ) } };
        const QLineF sector_center_line { position, position + 10.0 * QPointF { std::cos( radian_center ), std::sin( radian_center ) } };
        const QLineF sector_anchor_line { position, position - 10.0 * QPointF { std::cos( radian_center ), std::sin( radian_center ) } };
        const QLineF sector_end_line { position, position + 10.0 * QPointF { std::cos( radian_end ), std::sin( radian_end ) } };

        if( sector_center_line.intersects( left, &sector.intersection.center ) == QLineF::IntersectType::BoundedIntersection );
        else if( sector_center_line.intersects( top, &sector.intersection.center ) == QLineF::IntersectType::BoundedIntersection );
        else if( sector_center_line.intersects( right, &sector.intersection.center ) == QLineF::IntersectType::BoundedIntersection );
        else if( sector_center_line.intersects( bottom, &sector.intersection.center ) == QLineF::IntersectType::BoundedIntersection );
        else throw;

        if( sector_anchor_line.intersects( left, &sector.anchor ) == QLineF::IntersectType::BoundedIntersection );
        else if( sector_anchor_line.intersects( top, &sector.anchor ) == QLineF::IntersectType::BoundedIntersection );
        else if( sector_anchor_line.intersects( right, &sector.anchor ) == QLineF::IntersectType::BoundedIntersection );
        else if( sector_anchor_line.intersects( bottom, &sector.anchor ) == QLineF::IntersectType::BoundedIntersection );
        else throw;

        if( sector_begin_line.intersects( left, &sector.intersection.begin ) == QLineF::IntersectType::BoundedIntersection )
        {
            if( sector_end_line.intersects( left, &sector.intersection.end ) == QLineF::IntersectionType::BoundedIntersection )
            {
                sector.area = compute_area( position, sector.intersection.begin, sector.intersection.end );
                sector.length = sector.intersection.begin.y() - sector.intersection.end.y();
            }
            else if( sector_end_line.intersects( bottom, &sector.intersection.end ) == QLineF::IntersectionType::BoundedIntersection )
            {
                sector.area = compute_area( position, sector.intersection.begin, bottomleft ) + compute_area( position, sector.intersection.end, bottomleft );
                sector.length = sector.intersection.begin.y() - bottomleft.y() + sector.intersection.end.x() - bottomleft.x();
            }
            else if( sector_end_line.intersects( right, &sector.intersection.end ) == QLineF::IntersectionType::BoundedIntersection )
            {
                sector.area = compute_area( position, sector.intersection.begin, bottomleft ) + compute_area( position, bottomleft, bottomright ) + compute_area( position, sector.intersection.end, bottomright );
                sector.length = sector.intersection.begin.y() - bottomleft.y() + bottomright.x() - bottomleft.x() + sector.intersection.end.y() - bottomright.y();
            }
            else if( sector_end_line.intersects( top, &sector.intersection.end ) == QLineF::IntersectionType::BoundedIntersection )
            {
                throw;
            }
            else throw;
        }
        else if( sector_begin_line.intersects( top, &sector.intersection.begin ) == QLineF::IntersectType::BoundedIntersection )
        {
            if( sector_end_line.intersects( top, &sector.intersection.end ) == QLineF::IntersectionType::BoundedIntersection )
            {
                sector.area = compute_area( position, sector.intersection.begin, sector.intersection.end );
                sector.length = sector.intersection.begin.x() - sector.intersection.end.x();
            }
            else if( sector_end_line.intersects( left, &sector.intersection.end ) == QLineF::IntersectionType::BoundedIntersection )
            {
                sector.area = compute_area( position, sector.intersection.begin, topleft ) + compute_area( position, sector.intersection.end, topleft );
                sector.length = sector.intersection.begin.x() - topleft.x() + topleft.y() - sector.intersection.end.y();
            }
            else if( sector_end_line.intersects( bottom, &sector.intersection.end ) == QLineF::IntersectionType::BoundedIntersection )
            {
                sector.area = compute_area( position, sector.intersection.begin, topleft ) + compute_area( position, topleft, bottomleft ) + compute_area( position, sector.intersection.end, bottomleft );
                sector.length = sector.intersection.begin.x() - topleft.x() + topleft.y() - bottomleft.y() + sector.intersection.end.x() - bottomleft.x();
            }
            else if( sector_end_line.intersects( right, &sector.intersection.end ) == QLineF::IntersectionType::BoundedIntersection )
            {
                throw;
            }
            else throw;
        }
        else if( sector_begin_line.intersects( right, &sector.intersection.begin ) == QLineF::IntersectType::BoundedIntersection )
        {
            if( sector_end_line.intersects( right, &sector.intersection.end ) == QLineF::IntersectionType::BoundedIntersection )
            {
                sector.area = compute_area( position, sector.intersection.begin, sector.intersection.end );
                sector.length = sector.intersection.end.y() - sector.intersection.begin.y();
            }
            else if( sector_end_line.intersects( top, &sector.intersection.end ) == QLineF::IntersectionType::BoundedIntersection )
            {
                sector.area = compute_area( position, sector.intersection.begin, topright ) + compute_area( position, sector.intersection.end, topright );
                sector.length = topright.y() - sector.intersection.begin.y() + topright.x() - sector.intersection.end.x();
            }
            else if( sector_end_line.intersects( left, &sector.intersection.end ) == QLineF::IntersectionType::BoundedIntersection )
            {
                sector.area = compute_area( position, sector.intersection.begin, topright ) + compute_area( position, topright, topleft ) + compute_area( position, sector.intersection.end, topleft );
                sector.length = topright.y() - sector.intersection.begin.y() + topright.x() - topleft.x() + topleft.y() - sector.intersection.end.y();
            }
            else if( sector_end_line.intersects( bottom, &sector.intersection.end ) == QLineF::IntersectionType::BoundedIntersection )
            {
                throw;
            }
            else throw;
        }
        else if( sector_begin_line.intersects( bottom, &sector.intersection.begin ) == QLineF::IntersectType::BoundedIntersection )
        {
            if( sector_end_line.intersects( bottom, &sector.intersection.end ) == QLineF::IntersectionType::BoundedIntersection )
            {
                sector.area = compute_area( position, sector.intersection.begin, sector.intersection.end );
                sector.length = sector.intersection.end.x() - sector.intersection.begin.x();
            }
            else if( sector_end_line.intersects( right, &sector.intersection.end ) == QLineF::IntersectionType::BoundedIntersection )
            {
                sector.area = compute_area( position, sector.intersection.begin, bottomright ) + compute_area( position, sector.intersection.end, bottomright );
                sector.length = bottomright.x() - sector.intersection.begin.x() + sector.intersection.end.y() - bottomright.y();
            }
            else if( sector_end_line.intersects( top, &sector.intersection.end ) == QLineF::IntersectionType::BoundedIntersection )
            {
                sector.area = compute_area( position, sector.intersection.begin, bottomright ) + compute_area( position, bottomright, topright ) + compute_area( position, sector.intersection.end, topright );
                sector.length = bottomright.x() - sector.intersection.begin.x() + topright.y() - bottomright.y() + topright.x() - sector.intersection.end.x();
            }
            else if( sector_end_line.intersects( left, &sector.intersection.end ) == QLineF::IntersectionType::BoundedIntersection )
            {
                throw;
            }
            else throw;
        }
        else throw;

        // sector.length = QLineF { sector.intersection.begin, sector.intersection.end }.length();

        return sector;
    }
    void clamp( QPointF& point )
    {
        point.setX( std::clamp( point.x(), -0.99, 0.99 ) );
        point.setY( std::clamp( point.y(), -0.99, 0.99 ) );
    }
};

class Scatterplot
{
public:
    Scatterplot() noexcept = default;
    Scatterplot( const std::vector<QPointF>& points, size_t sectors ) : _points( points.size() )
    {
        for( size_t i = 0; i < points.size(); ++i )
        {
            _points[i].position = points[i];
            _points[i].sectors = std::vector<Sector>( sectors );
        }

        this->compute();
    }

    const auto& points() const noexcept
    {
        return _points;
    }
    const auto& domain() const noexcept
    {
        return _domain;
    }
    double computation_time() const noexcept
    {
        return _computation_time;
    }

    Scatterplot regularize()
    {
        std::vector<QPointF> points( _points.size() );

        double absmax = 0.0;
        for( size_t i = 0; i < _points.size(); ++i )
        {
            points[i] = _points[i].position + 0.85 * _points[i].deformation.total;
            _domain.clamp( points[i] );

            absmax = std::max( absmax, std::abs( points[i].x() ) );
            absmax = std::max( absmax, std::abs( points[i].y() ) );
        }

        return Scatterplot { std::move( points ), _points[0].sectors.size() };
    }

private:
    void compute()
    {
        const auto time_start = std::chrono::high_resolution_clock::now();

        for( size_t current_point_index = 0; current_point_index < _points.size(); ++current_point_index )
        {
            auto& current_point = _points[current_point_index];
            auto& sectors = current_point.sectors;
            const auto& current_position = current_point.position;

            // Compute sectors
            const auto sector_radian_step = 2.0 * std::numbers::pi_v<double> / sectors.size();
            for( uint32_t sector_index = 0; sector_index < sectors.size(); ++sector_index )
            {
                const double radian_begin = sector_index * sector_radian_step;
                const double radian_end = ( sector_index + 1.0 ) * sector_radian_step;
                sectors[sector_index] = _domain.sector( current_position, radian_begin, radian_end );
            }

            // Count sector points
            for( size_t other_point_index = 0; other_point_index < _points.size(); ++other_point_index )
            {
                if( current_point_index == other_point_index ) continue;

                const auto& other_position = _points[other_point_index].position;
                if( other_position == current_position )
                    continue;

                const auto direction = current_position - other_position;
                const auto radian = std::atan2( direction.y(), direction.x() );

                const auto t = std::clamp( ( radian + std::numbers::pi_v<double> ) / ( 2.0 * std::numbers::pi_v<double> ), 0.0, 1.0 );
                const auto sector_index = std::clamp( static_cast<size_t>( t * sectors.size() ), 0ull, sectors.size() - 1 );
                ++sectors[sector_index].points_count;
            }

            // Compute deformation
            current_point.deformation.density = QPointF { 0.0, 0.0 };
            current_point.deformation.uniform = QPointF { 0.0, 0.0 };
            current_point.deformation.boundary = QPointF { 0.0, 0.0 };

            for( auto& sector : sectors )
            {
                sector.deformation.density = sector.points_count / _points.size() * sector.anchor;
                sector.deformation.uniform = -sector.area / _domain.total_area() * sector.anchor;
                sector.deformation.boundary = -0.01 * sector.length / _domain.total_circumference() * sector.anchor;

                current_point.deformation.density += sector.deformation.density;
                current_point.deformation.uniform += sector.deformation.uniform;
                current_point.deformation.boundary += sector.deformation.boundary;
            }

            current_point.deformation.total = current_point.deformation.density + current_point.deformation.uniform; // + current_point.deformation.boundary;

            // if( QLineF { current_point.deformation.total, QPointF {} }.length() < 0.005 )
            //     current_point.deformation.total = QPointF {};
        }

        const auto time_end = std::chrono::high_resolution_clock::now();
        _computation_time = std::chrono::duration_cast<std::chrono::microseconds>( time_end - time_start ).count() / 1000.0;
        std::cout << "Finished computation in " << _computation_time << " ms." << std::endl;
    }

    struct Point
    {
        QPointF position {};

        std::vector<Sector> sectors {};

        struct
        {
            QPointF density {};
            QPointF boundary {};
            QPointF uniform {};
            QPointF total {};
        } deformation {};
    };

    std::vector<Point> _points {};
    SquareDomain _domain {};
    double _computation_time {};
};

class ScatterplotWidget : public QWidget
{
public:
    ScatterplotWidget() : QWidget {}
    {
        this->setFocusPolicy( Qt::WheelFocus );
        this->setFocus();

        std::normal_distribution<double> cluster_a { 0.0, 0.1 };
        std::normal_distribution<double> cluster_b { -0.7, 0.075 };
        std::normal_distribution<double> cluster_c { 0.4, 0.05 };

        std::mt19937_64 engine { 42 };

        for( uint64_t i {}; i < 200; ++i )
        {
            _original_points.push_back(
                QPointF {
                    std::clamp( cluster_a( engine ), -1.0, 1.0 ),
                    std::clamp( -cluster_c( engine ), -1.0, 1.0 )
                }
            );
            _labels.push_back( 0 );
        }

        for( uint64_t i {}; i < 350; ++i )
        {
            _original_points.push_back(
                QPointF {
                    std::clamp( cluster_c( engine ), -1.0, 1.0 ),
                    std::clamp( cluster_c( engine ), -1.0, 1.0 )
                }
            );
            _labels.push_back( 1 );
        }

        for( uint64_t i {}; i < 700; ++i )
        {
            _original_points.push_back(
                QPointF {
                    std::clamp( -cluster_c( engine ), -1.0, 1.0 ),
                    std::clamp( cluster_c( engine ), -1.0, 1.0 )
                }
            );
            _labels.push_back( 2 );
        }

        // _original_points = std::vector<QPointF> {
        //     QPointF { -0.99, -0.99 },
        //     QPointF { -0.99,  0.99 },
        //     QPointF {  0.99, -0.99 },
        //     QPointF {  0.99,  0.99 },
        // 
        //     QPointF { -0.5, 0.0 },
        //     // QPointF {  0.3, 0.5 },
        // };

        if( false )
        {
            _original_points.clear();
            _labels.clear();

            auto stream = std::ifstream { "../datasets/iris_embedding.csv" };
            stream.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );

            std::string line;
            while( std::getline( stream, line ) )
            {
                auto linestream = std::stringstream { line };

                double x;
                double y;
                uint32_t label = 0;
                ( ( linestream >> x ).ignore( 1 ) >> y ).ignore( 1 ) >> label;

                _original_points.push_back( QPointF { x, y } / 1.05 );
                _labels.push_back( label );
            }
        }

        if( _labels.size() != _original_points.size() )
            _labels = std::vector<uint32_t>( _original_points.size() );
    }

private:
    void paintEvent( QPaintEvent* event ) override
    {
        auto painter = QPainter { this };
        painter.setRenderHint( QPainter::Antialiasing, true );

        const auto point_size = 10.0;

        const auto default_font = painter.font();
        auto bold_font = painter.font();
        bold_font.setBold( true );
        bold_font.setPointSize( 20 );

        const auto radius = ( std::min( this->width(), this->height() ) - 50.0 ) / 2.0;
        const QPointF center = this->rect().center();
        const auto rectangle = QRectF { center - QPointF { radius, radius }, center + QPointF { radius, radius } };

        painter.setPen( QPen( Qt::lightGray, 1.0 ) );
        // painter.drawLine( center - QPointF { 5.0, 0.0 }, rectangle.center() + QPointF { 5.0, 0.0 } );
        // painter.drawLine( center - QPointF { 0.0, 5.0 }, rectangle.center() + QPointF { 0.0, 5.0 } );

        const auto& scatterplot = this->scatterplot( _sector_count, _iterations );
        const auto& sector_scatterplots = _scatterplots[_sector_count];

        if( _debug && !_render_all )
        {
            std::cout << "[ ---------------------------------------- Debug ---------------------------------------- ]" << std::endl;

            const auto& point = scatterplot.points()[_sample_index];
            const auto& sample_sectors = point.sectors;

            double area_sum = 0.0;
            double length_sum = 0.0;

            for( size_t i = 0; i < sample_sectors.size(); ++i )
            {
                const auto& sector = sample_sectors[i];

                const auto screen = center + radius * point.position;
                const auto intersection_begin = center + radius * sector.intersection.begin;
                const auto intersection_center = center + radius * sector.intersection.center;
                const auto intersection_end = center + radius * sector.intersection.end;

                area_sum += sector.area;
                length_sum += sector.length;

                if( _sector_colors )
                {
                    const QPolygonF polygon {
                        screen,
                        screen + 100.0 * ( intersection_begin - screen ),
                        screen + 100.0 * ( intersection_end - screen )
                    };
                    painter.setClipRect( rectangle );

                    const auto value = std::clamp( ( sector.points_count / scatterplot.points().size() - sector.area / scatterplot.domain().total_area() ) * 5.0, -1.0, 1.0 );
                    std::cout << value << std::endl;

                    auto color = value <= 0.0? QColor( 59, 76, 192 ) : QColor( 180, 4, 38 );
                    color.setAlpha( std::abs( value ) * 100 );
                    painter.setBrush( color );

                    painter.setPen( Qt::transparent );
                    painter.drawPolygon( polygon );

                    painter.setBrush( Qt::black );

                    painter.setClipRect( this->rect() );
                }

                painter.setPen( QPen( Qt::lightGray, 2.0 ) );
                painter.drawLine( screen, intersection_begin );
                painter.drawLine( screen, intersection_end );

                painter.setPen( QPen( Qt::lightGray, 2.0 ) );
                painter.drawLine( screen, intersection_end );

                // if( sector.points_count == 0.0 )
                //     continue;

                painter.setFont( bold_font );
                const auto label = QString::number( i );
                const auto label_width = painter.fontMetrics().horizontalAdvance( label );
                auto label_rectangle = QRectF { QPointF {}, QSizeF( label_width, painter.fontMetrics().height() ) };
                label_rectangle.moveCenter( 0.75 * intersection_center + 0.25 * screen );

                painter.setPen( QPen( Qt::black ) );
                // painter.drawText( label_rectangle, Qt::AlignCenter, label );

                // std::cout << "    Sample Index = " << _sample_index << ", Sector Index = " << i << std::endl;
                // std::cout << "        points = " << sector.points_count << ", area = " << sector.area << ", length = " << sector.length << std::endl;
                // std::cout << "        anchor = " << sector.anchor << ", density -> " << sector.deformation.density << ", boundary -> " << sector.deformation.boundary << ", uniform -> " << sector.deformation.uniform << std::endl;
            }

            std::cout << "Sample position        = " << point.position << std::endl;
            std::cout << "Sum of areas           = " << area_sum << std::endl;
            std::cout << "Sum of lengths         = " << length_sum << std::endl;
            std::cout << "Deformation (density)  = " << point.deformation.density << std::endl;
            std::cout << "Deformation (boundary) = " << point.deformation.boundary << std::endl;
            std::cout << "Deformation (uniform)  = " << point.deformation.uniform << std::endl;
            std::cout << "Deformation (total)    = " << point.deformation.total << std::endl;
        }

        painter.setPen( QPen( Qt::lightGray, 2.0 ) );
        painter.setBrush( Qt::transparent );
        painter.drawRect( rectangle );

        double absmax = 0.0;
        for( const auto& point : scatterplot.points() )
        {
            absmax = std::max( absmax, point.position.x() );
            absmax = std::max( absmax, point.position.y() );
        }

        painter.setPen( QPen( Qt::black, 1.0 ) );
        for( size_t i = 0; i < scatterplot.points().size(); ++i )
        {
            const auto& point = scatterplot.points()[i];
            const auto screen = center + radius * ( _normalize ? point.position / ( absmax / 0.99 ) : point.position );

            if( _debug && ( _render_all || i == _sample_index ) )
            {
                const auto width = _render_all ? 1.0 : 3.0;
                const auto alpha = _render_all? 50 : 255;
                painter.setPen( QPen( QColor { 63, 100, 127, alpha }, width ) ); // blue
                painter.drawLine( screen, screen + radius * 0.85 * point.deformation.total );

                if( !_render_all )
                {
                    painter.setPen( QPen( QColor { 255, 0, 0 }, width ) ); // red
                    // painter.drawLine( screen, screen + radius * 0.85 * point.deformation.density );

                    painter.setPen( QPen( QColor { 252, 186, 3 }, width ) ); // yellow
                    // painter.drawLine( screen, screen + radius * 0.85 * point.deformation.boundary );

                    painter.setPen( QPen( QColor { 0, 255, 0 }, width ) ); // green
                    // painter.drawLine( screen, screen + radius * 0.85 * point.deformation.uniform );

                    painter.setBrush( Qt::lightGray );
                    for( const auto& sector : point.sectors )
                    {
                        const auto anchor = center + radius * sector.anchor;
                        // painter.setPen( QPen { Qt::lightGray, 2.0, Qt::DashLine } );
                        // painter.drawLine( screen, anchor );

                        painter.setPen( Qt::black );
                        // painter.drawEllipse( anchor, point_size, point_size );
                    }
                    painter.setBrush( Qt::lightGray );
                }

                painter.setPen( QPen( Qt::black, 1.0 ) );
            }

            painter.setBrush( _colors[_labels[i]] );
            painter.drawEllipse( screen, point_size, point_size );
        }

        const auto render_path = [this, &painter, center, radius, point_size, &sector_scatterplots] ( size_t point_index, double width, bool render_checkpoints )
        {
            for( uint32_t j = 1; j <= _iterations; ++j )
            {
                const auto previous = center + radius * sector_scatterplots[j - 1].points()[point_index].position;
                const auto current = center + radius * sector_scatterplots[j].points()[point_index].position;

                painter.setPen( QPen( QColor { 63, 100, 127, 255 }, width, Qt::DashLine ) );
                painter.drawLine( previous, current );
                
                if( render_checkpoints )
                {
                    painter.setPen( Qt::transparent );
                    painter.setBrush( QColor { 63, 100, 127, 255 } );
                    painter.drawEllipse( previous, point_size / 2.0, point_size / 2.0 );
                }
            }
        };

        // Render point paths
        if( _debug )
        {
            if( _render_all )
            {
                for( size_t i = 0; i < scatterplot.points().size(); ++i )
                    render_path( i, 1.0, false );
            }
            else
            {
                render_path( _sample_index, 3.0, true );
            }
        }

        const auto text = "Iterations: " + QString::number( _iterations ) + "\nSectors: " + QString::number( _sector_count );
        auto text_rectangle = this->rect().marginsRemoved( QMargins { 10, 10, 10, 10 } );
        text_rectangle.moveRight( rectangle.left() - 10 );

        painter.setFont( bold_font );
        painter.setPen( QPen( Qt::black, 2.0 ) );
        painter.drawText( text_rectangle, Qt::AlignRight | Qt::AlignTop, text );

        if( false && _debug && !_render_all )
        {
            auto debug_text = QString {};
            const auto& sample_sectors = scatterplot.points()[_sample_index].sectors;
            for( size_t i = 0; i < sample_sectors.size(); ++i )
            {
                const auto& sector = sample_sectors[i];
                debug_text += "Sector #" + QString::number( i ) + ": P = " + QString::number( sector.points_count ) + ", A = " + QString::number( sector.area ) + ", L = " + QString::number( sector.length ) + "\n";
            }

            auto debug_text_rectangle = text_rectangle;
            debug_text_rectangle.moveTop( rectangle.top() + 2.0 * painter.fontMetrics().height() + 10.0 );

            auto font = painter.font();
            font.setBold( false );
            font.setPointSize( 14 );
            painter.setFont( font );

            painter.setPen( QPen( Qt::black ) );
            painter.drawText( debug_text_rectangle, Qt::AlignLeft | Qt::AlignTop, debug_text );
            painter.setFont( default_font );

            // painter.setPen( Qt::transparent );
            // painter.setBrush( QColor { 235, 64, 52 } ); // red
            // painter.drawEllipse( center + radius * QPointF { -1.0, -1.0 }, 5.0, 5.0 );
        }
    }

    void mousePressEvent( QMouseEvent* event )
    {
        if( event->button() == Qt::LeftButton )
        {
            struct
            {
                size_t index { std::numeric_limits<size_t>::max() };
                double distance { 10.0 };
            } closest;

            uint64_t close_points_counter = 0;

            const auto radius = ( std::min( this->width(), this->height() ) - 20.0 ) / 2.0;
            const QPointF center = this->rect().center();
            const auto& points = this->scatterplot( _sector_count, _iterations ).points();

            for( size_t i = 0; i < points.size(); ++i )
            {
                const auto screen = center + radius * points[i].position;
                const auto distance = QLineF { event->localPos(), screen }.length();
                if( distance < 10.0 )
                    ++close_points_counter;
                if( distance < closest.distance )
                    closest = { i, distance };
            }

            std::cout << "Number of points close to cursor: " << close_points_counter << std::endl;

            if( closest.index != std::numeric_limits<size_t>::max() )
            {
                _sample_index = closest.index;
                this->update();
            }
        }
    }
    void wheelEvent( QWheelEvent* event ) override
    {
        if( event->modifiers() & Qt::ShiftModifier )
        {
            if( event->angleDelta().y() > 0 )
                ++_sector_count;
            else if( _sector_count > 2 )
                --_sector_count;
        }
        else
        {
            if( event->angleDelta().y() > 0 )
                _iterations += ( event->modifiers() & Qt::ControlModifier ) ? 10 : 1;
            else
                _iterations = std::max( 0ll, _iterations - ( event->modifiers() & Qt::ControlModifier ? 10 : 1 ) );
        }

        this->update();
    }
    void keyPressEvent( QKeyEvent* event ) override
    {
        if( event->key() == Qt::Key_R )
        {
            _iterations = 0;
            this->update();
        }
        else if( event->key() == Qt::Key_D )
        {
            _debug = !_debug;
            this->update();
        }
        else if( event->key() == Qt::Key_A )
        {
            _render_all = !_render_all;
            this->update();
        }
        else if( event->key() == Qt::Key_P )
        {
            _render_path = !_render_path;
            this->update();
        }
        else if( event->key() == Qt::Key_N )
        {
            _normalize = !_normalize;
            this->update();
        }
        else if( event->key() == Qt::Key_C )
        {
            _sector_colors = !_sector_colors;
            this->update();
        }
        else if( event->key() == Qt::Key_E )
        {
            for( const auto sector_count : { 4, 8, 18, 36, 72, 180, 360, 720 } )
            {
                for( const auto iterations : { 0, 1, 2, 4, 8, 16, 32, 64, 128, 256 } )
                {
                    const auto filepath = "results/square_evaluation_s" + std::to_string( sector_count ) + "_i" + std::to_string( iterations ) + ".csv";
                    auto stream = std::ofstream { filepath };
                    stream << "time,point_index,x,y,sector_index,point_count,area,length\n";

                    std::cout << filepath << std::endl;

                    const auto& scatterplot = this->scatterplot( sector_count, iterations );
                    for( size_t point_index = 0; point_index < scatterplot.points().size(); ++point_index )
                    {
                        const auto& point = scatterplot.points()[point_index];
                        for( size_t sector_index = 0; sector_index < sector_count; ++sector_index )
                        {
                            const auto& sector = point.sectors[sector_index];
                            stream << sector_count << ',' << iterations << ',' << scatterplot.computation_time() << ','
                                << point_index << ',' << point.position.x() << ',' << point.position.y() << ','
                                << sector_index << ',' << sector.points_count << ',' << sector.area << ',' << sector.length << '\n';
                        }
                    }
                }

                _scatterplots.clear();
            }
        }
    }

    const Scatterplot& scatterplot( size_t sector_count, size_t iterations )
    {
        auto& scatterplots = _scatterplots[sector_count];
        if( scatterplots.empty() )
            scatterplots.push_back( Scatterplot { _original_points, sector_count } );

        while( iterations >= scatterplots.size() )
            scatterplots.push_back( scatterplots.back().regularize() );
        return scatterplots[iterations];
    }

    std::vector<QPointF> _original_points {};
    std::vector<uint32_t> _labels {};
    const std::vector<QColor> _colors {
        QColor { "#ffc700" },
        QColor { "#ffa42b" },
        QColor { "#00c2f9" },
        QColor { "#0f718d" }
    };

    std::unordered_map<size_t, std::vector<Scatterplot>> _scatterplots {};
    size_t _sector_count { 16 };
    int64_t _iterations { 0 };
    size_t _sample_index { 0 };

    bool _debug { false };
    bool _render_all { false };
    bool _render_path { false };
    bool _normalize { false };
    bool _sector_colors { false };
};

int main( int argc, char** argv )
{
    auto application = QApplication { argc, argv };

    auto window = QWidget {};
    window.resize( 1920, 1080 );
    window.setWindowTitle( "Sector-based Scatterplot De-cluttering" );
    window.setStyleSheet( "background: white" );
    window.show();

    auto scatterplotWidget = new ScatterplotWidget {};

    auto layout = new QVBoxLayout { &window };
    layout->addWidget( scatterplotWidget );

    return application.exec();
}