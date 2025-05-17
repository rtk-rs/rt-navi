use ratatui::{
    buffer::Buffer,
    layout::{Constraint, Layout, Rect},
    style::{Color, Modifier, Style, Stylize},
    symbols,
    text::{Line, Span},
    widgets::{
        canvas::{Canvas, Circle, Map, MapResolution},
        Axis, Block, Cell, Chart, Dataset, GraphType, Paragraph, Row, Table, Tabs, Widget, Wrap,
    },
    Frame,
};

use crate::rt_navi::RtNavi;



pub fn draw(frame: &mut Frame, app: &mut RtNavi) {
    let chunks = Layout::vertical([Constraint::Length(3), Constraint::Min(0)]).split(frame.area());

    let tabs = app
        .tabs
        .titles
        .iter()
        .map(|t| Line::from(Span::styled(*t, Style::default().fg(Color::Green))))
        .collect::<Tabs>()
        .block(Block::bordered().title(app.title))
        .highlight_style(Style::default().fg(Color::Yellow))
        .select(app.tabs.index);

    frame.render_widget(tabs, chunks[0]);

    match app.tabs.index {
        0 => draw_map(frame, app, chunks[1]),
        1 => draw_charts(frame, app, chunks[1]),
        _ => {},
    };
}

fn draw_pvt_tab(frame: &mut Frame, app: &mut RtNavi, area: Rect) {
    let chunks = Layout::vertical([Constraint::Length(24), Constraint::Min(7)]).split(area);
    render_pvt_state(frame, chunks[0], app);
    frame.render_widget(&mut app.log_widget, chunks[1]);
}

fn draw_esf_tab(frame: &mut Frame, app: &mut RtNavi, area: Rect) {
    let chunks = Layout::vertical([Constraint::Length(24), Constraint::Min(7)]).split(area);
    render_esf_status(frame, chunks[0], app);
    frame.render_widget(&mut app.log_widget, chunks[1]);
}

fn draw_esf_charts_tab(frame: &mut Frame, app: &mut App, area: Rect) {
    render_sensor_charts(frame, area, app);
}

fn draw_version_info(frame: &mut Frame, app: &mut App, area: Rect) {
    let chunks = Layout::vertical([Constraint::Length(24), Constraint::Min(7)]).split(area);
    render_monver(frame, chunks[0], app);
    frame.render_widget(&mut app.log_widget, chunks[1]);
}


fn render_speed_chart(frame: &mut Frame, area: Rect, app: &mut App) {
    let x_mean = (app.signals.speed.x_bounds[0] + app.signals.speed.x_bounds[1]) / 2.0;
    let y_mean = (app.signals.speed.y_bounds[0] + app.signals.speed.y_bounds[1]) / 2.0;
    let x_labels = vec![
        Span::styled(
            format!("{:.2}", app.signals.speed.x_bounds[0]),
            Style::default().add_modifier(Modifier::BOLD),
        ),
        Span::raw(format!("{:.2}", x_mean)),
        Span::styled(
            format!("{:.2}", app.signals.speed.x_bounds[1]),
            Style::default().add_modifier(Modifier::BOLD),
        ),
    ];
    let y_labels = vec![
        Span::styled(
            format!("{:.2}", app.signals.speed.y_bounds[0]),
            Style::default().add_modifier(Modifier::BOLD),
        ),
        Span::raw(format!("{:.2}", y_mean)),
        Span::styled(
            format!("{:.2}", app.signals.speed.y_bounds[1]),
            Style::default().add_modifier(Modifier::BOLD),
        ),
    ];

    let datasets = vec![Dataset::default()
        .name("Speed")
        .marker(symbols::Marker::Dot)
        .style(Style::default().fg(Color::Cyan))
        .graph_type(GraphType::Line)
        .data(&app.signals.speed.points)];

    let speed = app.signals.speed.current();
    let title = format!("Speed: {:8.4} [m/s] / {:8.4} [km/h] ", speed, speed * 3.6);

    let chart = Chart::new(datasets)
        .block(Block::bordered())
        .x_axis(
            Axis::default()
                .title("Time [sec]")
                .style(Style::default().fg(Color::Gray))
                .labels(x_labels)
                .bounds(app.signals.speed.x_bounds),
        )
        .y_axis(
            Axis::default()
                .title(title)
                .style(Style::default().fg(Color::Gray))
                .labels(y_labels)
                .bounds(app.signals.speed.y_bounds),
        );

    frame.render_widget(chart, area);
}

fn render_acc_chart(frame: &mut Frame, area: Rect, app: &mut App) {
    let x_min_xy = f64::min(app.signals.acc_x.x_bounds[0], app.signals.acc_y.x_bounds[0]);
    let x_min_xyz = f64::min(x_min_xy, app.signals.acc_z.x_bounds[0]);
    let x_max_xy = f64::max(app.signals.acc_x.x_bounds[1], app.signals.acc_y.x_bounds[1]);
    let x_max_xyz = f64::max(x_max_xy, app.signals.acc_z.x_bounds[1]);
    let x_mean = (x_min_xyz + x_max_xyz) / 2.0;
    let x_labels = vec![
        Span::styled(
            format!("{:.2}", x_min_xyz),
            Style::default().add_modifier(Modifier::BOLD),
        ),
        Span::raw(format!("{:.2}", x_mean)),
        Span::styled(
            format!("{:.2}", x_max_xyz),
            Style::default().add_modifier(Modifier::BOLD),
        ),
    ];

    let y_min_xy = f64::min(app.signals.acc_x.y_bounds[0], app.signals.acc_y.y_bounds[0]);
    let y_min_xyz = f64::min(y_min_xy, app.signals.acc_z.y_bounds[0]);
    let y_max_xy = f64::max(app.signals.acc_x.y_bounds[1], app.signals.acc_y.y_bounds[1]);
    let y_max_xyz = f64::max(y_max_xy, app.signals.acc_z.y_bounds[1]);
    let y_mean = (y_min_xyz + y_max_xyz) / 2.0;
    let y_labels = vec![
        Span::styled(
            format!("{:.2}", y_min_xyz * 0.9),
            Style::default().add_modifier(Modifier::BOLD),
        ),
        Span::raw(format!("{:.2}", y_mean)),
        Span::styled(
            format!("{:.2}", y_max_xyz * 1.1),
            Style::default().add_modifier(Modifier::BOLD),
        ),
    ];
    let datasets = vec![
        Dataset::default()
            .name("AccX")
            .marker(symbols::Marker::Dot)
            .style(Style::default().fg(Color::Cyan))
            .graph_type(GraphType::Line)
            .data(&app.signals.acc_x.points),
        Dataset::default()
            .name("AccY")
            .marker(symbols::Marker::Dot)
            .graph_type(GraphType::Line)
            .style(Style::default().fg(Color::Yellow))
            .data(&app.signals.acc_y.points),
        Dataset::default()
            .name("AccZ")
            .marker(symbols::Marker::Dot)
            .graph_type(GraphType::Line)
            .style(Style::default().fg(Color::Red))
            .data(&app.signals.acc_z.points),
    ];

    let x = app.signals.acc_x.current();
    let y = app.signals.acc_y.current();
    let z = app.signals.acc_z.current();

    let title = format!(
        "AccX: {:7.4}, AccY: {:7.4}, AccZ: {:7.4} [m/s^2]  ",
        x, y, z
    );
    let chart = Chart::new(datasets)
        .block(Block::bordered())
        .x_axis(
            Axis::default()
                .title("Time [sec]")
                .style(Style::default().fg(Color::Gray))
                .labels(x_labels)
                .bounds([x_min_xyz, x_max_xyz]),
        )
        .y_axis(
            Axis::default()
                .title(title)
                .style(Style::default().fg(Color::Gray))
                .labels(y_labels)
                .bounds([y_min_xyz, y_max_xyz]),
        );

    frame.render_widget(chart, area);
}

fn render_gyro_chart(frame: &mut Frame, area: Rect, app: &mut App) {
    let x_min_xy = f64::min(
        app.signals.gyro_x.x_bounds[0],
        app.signals.gyro_y.x_bounds[0],
    );
    let x_min_xyz = f64::min(x_min_xy, app.signals.gyro_z.x_bounds[0]);
    let x_max_xy = f64::max(
        app.signals.gyro_x.x_bounds[1],
        app.signals.gyro_y.x_bounds[1],
    );
    let x_max_xyz = f64::max(x_max_xy, app.signals.gyro_z.x_bounds[1]);
    let x_mean = (x_min_xyz + x_max_xyz) / 2.0;
    let x_labels = vec![
        Span::styled(
            format!("{:.2}", x_min_xyz),
            Style::default().add_modifier(Modifier::BOLD),
        ),
        Span::raw(format!("{:.2}", x_mean)),
        Span::styled(
            format!("{:.2}", x_max_xyz),
            Style::default().add_modifier(Modifier::BOLD),
        ),
    ];

    let y_min_xy = f64::min(
        app.signals.gyro_x.y_bounds[0],
        app.signals.gyro_y.y_bounds[0],
    );
    let y_min_xyz = f64::min(y_min_xy, app.signals.gyro_z.y_bounds[0]);
    let y_max_xy = f64::max(
        app.signals.gyro_x.y_bounds[1],
        app.signals.gyro_y.y_bounds[1],
    );
    let y_max_xyz = f64::max(y_max_xy, app.signals.gyro_z.y_bounds[1]);
    let y_mean = (y_min_xyz + y_max_xyz) / 2.0;
    let y_labels = vec![
        Span::styled(
            format!("{:.2}", y_min_xyz),
            Style::default().add_modifier(Modifier::BOLD),
        ),
        Span::raw(format!("{:.2}", y_mean)),
        Span::styled(
            format!("{:.2}", y_max_xyz),
            Style::default().add_modifier(Modifier::BOLD),
        ),
    ];
    let datasets = vec![
        Dataset::default()
            .name("GyroX")
            .marker(symbols::Marker::Dot)
            .style(Style::default().fg(Color::Cyan))
            .graph_type(GraphType::Line)
            .data(&app.signals.gyro_x.points),
        Dataset::default()
            .name("GyroY")
            .marker(symbols::Marker::Dot)
            .style(Style::default().fg(Color::Yellow))
            .graph_type(GraphType::Line)
            .data(&app.signals.gyro_y.points),
        Dataset::default()
            .name("GyroZ")
            .marker(symbols::Marker::Dot)
            .style(Style::default().fg(Color::Red))
            .graph_type(GraphType::Line)
            .data(&app.signals.gyro_z.points),
    ];

    let x = app.signals.gyro_x.current();
    let y = app.signals.gyro_y.current();
    let z = app.signals.gyro_z.current();

    let title = format!(
        "GyroX: {:7.4}, GyroY: {:7.4}, GyroZ: {:7.4} [deg/s]",
        x, y, z
    );
    let chart = Chart::new(datasets)
        .block(Block::bordered())
        .x_axis(
            Axis::default()
                .title("Time [sec]")
                .style(Style::default().fg(Color::Gray))
                .labels(x_labels)
                .bounds([x_min_xyz, x_max_xyz]),
        )
        .y_axis(
            Axis::default()
                .title(title)
                .style(Style::default().fg(Color::Gray))
                .labels(y_labels)
                .bounds([y_min_xyz, y_max_xyz]),
        );

    frame.render_widget(chart, area);
}

fn render_wheeltick_chart(frame: &mut Frame, area: Rect, app: &mut App) {
    let x_min_f = f64::min(app.signals.wt_fl.x_bounds[0], app.signals.wt_fr.x_bounds[0]);
    let x_min_r = f64::min(app.signals.wt_rl.x_bounds[0], app.signals.wt_rr.x_bounds[0]);
    let x_min_fr = f64::min(x_min_f, x_min_r);

    let x_max_f = f64::max(app.signals.wt_fl.x_bounds[1], app.signals.wt_fr.x_bounds[1]);
    let x_max_r = f64::max(app.signals.wt_rl.x_bounds[1], app.signals.wt_rr.x_bounds[1]);
    let x_max_fr = f64::max(x_max_f, x_max_r);
    let x_mean = (x_min_fr + x_max_fr) / 2.0;
    let x_labels = vec![
        Span::styled(
            format!("{:.2}", x_min_fr),
            Style::default().add_modifier(Modifier::BOLD),
        ),
        Span::raw(format!("{:.2}", x_mean)),
        Span::styled(
            format!("{:.2}", x_max_fr),
            Style::default().add_modifier(Modifier::BOLD),
        ),
    ];

    let y_min_f = f64::min(app.signals.wt_fl.y_bounds[0], app.signals.wt_fr.y_bounds[0]);
    let y_min_r = f64::min(app.signals.wt_rl.y_bounds[0], app.signals.wt_rr.y_bounds[0]);
    let y_min_fr = f64::min(
        f64::min(y_min_f, y_min_r),
        app.signals.speed_tick.y_bounds[0],
    );

    let y_max_f = f64::max(app.signals.wt_fl.y_bounds[1], app.signals.wt_fr.y_bounds[1]);
    let y_max_r = f64::max(app.signals.wt_rl.y_bounds[1], app.signals.wt_rr.y_bounds[1]);
    let y_max_fr = f64::max(
        f64::max(y_max_f, y_max_r),
        app.signals.speed_tick.y_bounds[1],
    );
    let y_mean = (y_min_fr + y_max_fr) / 2.0;
    let y_labels = vec![
        Span::styled(
            format!("{:.2}", y_min_fr),
            Style::default().add_modifier(Modifier::BOLD),
        ),
        Span::raw(format!("{:.2}", y_mean)),
        Span::styled(
            format!("{:.2}", y_max_fr),
            Style::default().add_modifier(Modifier::BOLD),
        ),
    ];

    let datasets = vec![
        Dataset::default()
            .name("WT-FL")
            .marker(symbols::Marker::Dot)
            .style(Style::default().fg(Color::Cyan))
            .graph_type(GraphType::Line)
            .data(&app.signals.wt_fl.points),
        Dataset::default()
            .name("WT-FR")
            .marker(symbols::Marker::Dot)
            .style(Style::default().fg(Color::Yellow))
            .graph_type(GraphType::Line)
            .data(&app.signals.wt_fr.points),
        Dataset::default()
            .name("WT-RL")
            .marker(symbols::Marker::Dot)
            .style(Style::default().fg(Color::Red))
            .graph_type(GraphType::Line)
            .data(&app.signals.wt_rl.points),
        Dataset::default()
            .name("WT-RR")
            .marker(symbols::Marker::Dot)
            .style(Style::default().fg(Color::Red))
            .graph_type(GraphType::Line)
            .data(&app.signals.wt_rr.points),
        Dataset::default()
            .name("Speed-Tick")
            .marker(symbols::Marker::Dot)
            .style(Style::default().fg(Color::Red))
            .graph_type(GraphType::Line)
            .data(&app.signals.speed_tick.points),
    ];

    let chart = Chart::new(datasets)
        .block(Block::bordered())
        .x_axis(
            Axis::default()
                .title("Time [sec]")
                .style(Style::default().fg(Color::Gray))
                .labels(x_labels)
                .bounds([x_min_fr, x_max_fr]),
        )
        .y_axis(
            Axis::default()
                .title("Wheel-Ticks")
                .style(Style::default().fg(Color::Gray))
                .labels(y_labels)
                .bounds([y_min_fr, y_max_fr]),
        );

    frame.render_widget(chart, area);
}

fn render_monver(frame: &mut Frame, area: Rect, app: &mut App) {
    let extensions_src = app.mon_ver_state.extensions.clone();

    let mut extensions_lines = Vec::new();
    let mut extensions = extensions_src.to_string();
    let mut extensions = if let Some(p) = extensions.find("FWVER") {
        let suffix = extensions.split_off(p);
        extensions_lines.push(Line::from(extensions));
        suffix
    } else {
        String::default()
    };

    let mut extensions = if let Some(p) = extensions.find("PROTVER") {
        let suffix = extensions.split_off(p);
        extensions_lines.push(Line::from(extensions));
        suffix
    } else {
        String::default()
    };

    let mut extensions = if let Some(p) = extensions.find("MOD") {
        let suffix = extensions.split_off(p);
        extensions_lines.push(Line::from(extensions));
        suffix
    } else {
        String::default()
    };

    let mut extensions = if let Some(p) = extensions.find("FIS") {
        let suffix = extensions.split_off(p);
        extensions_lines.push(Line::from(extensions));
        suffix
    } else {
        String::default()
    };

    let extensions = if let Some(p) = extensions.find(")") {
        let suffix = extensions.split_off(p + 1);
        extensions_lines.push(Line::from(extensions));
        suffix
    } else {
        String::default()
    };

    // Remaining content of extensions string
    extensions_lines.push(Line::from(extensions));

    let software_version = std::str::from_utf8(&app.mon_ver_state.software_version).unwrap();
    let hardware_version = std::str::from_utf8(&app.mon_ver_state.hardware_version).unwrap();

    let mut text = vec![
        Line::from(Span::styled(
            "Software Version",
            Style::default().fg(Color::Red),
        )),
        Line::from(vec![Span::from(" "), Span::from(software_version)]),
        Line::from(""),
        Line::from(Span::styled(
            "Hardware Version",
            Style::default().fg(Color::Red),
        )),
        Line::from(vec![Span::raw(""), Span::from(hardware_version)]),
        Line::from(""),
        Line::from(Span::styled(
            "Extensions",
            Style::default().fg(Color::Yellow),
        )),
    ];
    text.append(&mut extensions_lines);

    let mut raw_extensions = vec![
        Line::from(""),
        Line::from("Extensions as raw string:"),
        Line::from(extensions_src),
    ];

    text.append(&mut raw_extensions);

    let block = Block::bordered().title(Span::styled(
        "MON-VERSION",
        Style::default()
            .fg(Color::Magenta)
            .add_modifier(Modifier::BOLD),
    ));
    let paragraph = Paragraph::new(text).block(block).wrap(Wrap { trim: true });
    frame.render_widget(paragraph, area);
}

fn draw_map(frame: &mut Frame, app: &mut RtNavi, area: Rect) {

    let map = Canvas::default()
        .block(Block::bordered().title("World"))
        .paint(|ctx| {
            ctx.draw(&Map {
                color: Color::White,
                resolution: MapResolution::High,
            });
            ctx.layer();

            ctx.draw(&Circle {
                x: app.pvt_state.lon,
                y: app.pvt_state.lat,
                radius: 10.0,
                color: Color::Green,
            });

            ctx.print(
                app.pvt_state.lon,
                app.pvt_state.lat,
                Span::styled("X", Style::default().fg(Color::Green)),
            );
        })
        .marker(symbols::Marker::Braille)
        .x_bounds([-180.0, 180.0])
        .y_bounds([-90.0, 90.0]);
    
    frame.render_widget(map, area);
}
