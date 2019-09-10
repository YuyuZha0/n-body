(function (odex) {

    'use strict';

    const ODE_DIM = 12;
    const TOLERANCE = 1e-12;

    class Vector2D {

        constructor(a) {
            this.a = a;
        }

        static fromPolar(radius, angle) {
            const a = new Float64Array(3);
            a[0] = radius * Math.cos(angle);
            a[1] = radius * Math.sin(angle);
            a[2] = radius;
            return new Vector2D(a);
        }

        static fromCartesian(x, y) {
            const a = new Float64Array(3);
            a[0] = x;
            a[1] = y;
            a[2] = Math.hypot(x, y);
            return new Vector2D(a);
        }

        add(v) {
            return Vector2D.fromCartesian(this.a[0] + v.a[0], this.a[1] + v.a[1]);
        }

        sub(v) {
            return Vector2D.fromCartesian(this.a[0] - v.a[0], this.a[1] - v.a[1]);
        }

        len() {
            return this.a[2];
        }

        dot(v) {
            return this.a[0] * v.a[0] + this.a[1] * v.a[1];
        }

        gravity(m) {
            const l = this.a[2];
            const k = m / (l * l * l);
            const a = new Float64Array(3);
            a[0] = this.a[0] * k;
            a[1] = this.a[1] * k;
            a[2] = this.a[2] * k;
            return new Vector2D(a);
        }

        projectOn(v) {
            return this.dot(v) / v.len();
        }

        toString() {
            return this.a.join(',');
        }

    }

    const threeBodySystem = function (m1, m2, m3) {
        const PI_2 = Math.PI / 2;
        return function (t, u) {
            const r1 = Vector2D.fromPolar(u[0], u[1]),
                r2 = Vector2D.fromPolar(u[4], u[5]),
                r3 = Vector2D.fromPolar(u[8], u[9]);
            const a1 = Vector2D.fromPolar(1, u[1] + PI_2),
                a2 = Vector2D.fromPolar(1, u[5] + PI_2),
                a3 = Vector2D.fromPolar(1, u[9] + PI_2);
            const r12 = r1.sub(r2), r23 = r2.sub(r3), r31 = r3.sub(r1);
            const F12 = r12.gravity(m1 * m2), F23 = r23.gravity(m2 * m3), F31 = r31.gravity(m3 * m1);
            const F1 = F31.sub(F12), F2 = F12.sub(F23), F3 = F23.sub(F31);
            const v = new Array(ODE_DIM);
            v[0] = u[2];
            v[1] = u[3];
            v[2] = u[0] * u[3] * u[3] + F1.projectOn(r1) / m1;
            v[3] = -(2 * u[2] * u[3] - F1.projectOn(a1) / m1) / u[0];
            v[4] = u[6];
            v[5] = u[7];
            v[6] = u[4] * u[7] * u[7] + F2.projectOn(r2) / m2;
            v[7] = -(2 * u[6] * u[7] - F2.projectOn(a2) / m2) / u[4];
            v[8] = u[10];
            v[9] = u[11];
            v[10] = u[8] * u[11] * u[11] + F3.projectOn(r3) / m3;
            v[11] = -(2 * u[10] * u[11] - F3.projectOn(a3) / m3) / u[8];
            return v;
        };
    };


    const y0 = function (bodies) {
        const a = [];
        pushBodyInfo(bodies[0], a);
        pushBodyInfo(bodies[1], a);
        pushBodyInfo(bodies[2], a);
        return a;
    };

    const pushBodyInfo = function (body, a) {
        a.push(body.radius);
        a.push(body.angle);
        a.push(body.radiusSpeed);
        a.push(body.angleSpeed);
    };

    const toPaintableVector = function (y, bodies) {
        return [
            mkVec(0, y, bodies[0].mass),
            mkVec(4, y, bodies[1].mass),
            mkVec(8, y, bodies[2].mass)
        ];
    };

    const mkVec = function (offset, y, m) {
        const a = new Array(3);
        a[0] = y[offset];
        a[1] = toStdDegree(y[offset + 1]);
        a[2] = m;
        return a;
    };

    window.readBodyParam = function (bodies) {
        return toPaintableVector(y0(bodies), bodies);
    };

    const toStdDegree = function (x) {
        const d = x * 180 / Math.PI;
        return d - 360 * Math.floor(d / 360);
    };

    window.threeBodySimulator = function (options, renderer) {

        const defaultOptions = {
            bodies: [{
                mass: 80,
                radius: 7,
                angle: 0,
                radiusSpeed: 0,
                angleSpeed: 0.4
            }, {
                mass: 80,
                radius: 7,
                angle: Math.PI * 2 / 3,
                radiusSpeed: 0,
                angleSpeed: 0.3
            }, {
                mass: 80,
                radius: 7,
                angle: Math.PI * 4 / 3,
                radiusSpeed: 0,
                angleSpeed: 0.2
            }],
            maxTraceKeep: 1500,
            timeDelta: 0.005
        };
        options = Object.assign(defaultOptions, options);

        console.info("load options:", options);

        if (typeof renderer !== 'function' || !renderer instanceof Function) {
            throw Error('renderer must be a function!');
        }

        const solver = new odex.Solver(ODE_DIM);
        solver.absoluteTolerance = solver.relativeTolerance = TOLERANCE;

        const bodies = options.bodies;
        const f = threeBodySystem(bodies[0].mass, bodies[1].mass, bodies[2].mass);
        let y = y0(bodies), y1;
        let t = 0;

        const traces = [[], [], []];

        return function () {

            y1 = solver.solve(f, t, y, t + options.timeDelta).y;

            if (Number.isNaN(y1[0])
                || Number.isNaN(y1[4])
                || Number.isNaN(y1[8])) {
                console.info("terminated for collision, current time:%f", t);
                return;
            }

            const vectors = toPaintableVector(y, bodies);

            if (traces[0].length > options.maxTraceKeep) {
                traces[0].shift();
                traces[1].shift();
                traces[2].shift();
            }
            traces[0].push(vectors[0]);
            traces[1].push(vectors[1]);
            traces[2].push(vectors[2]);

            y = y1;
            t += options.timeDelta;

            renderer(vectors, traces);
        }
    };

})(require('odex'));

(function (echarts) {

    'use strict';

    const initChart = function (domId) {
        const domElement = document.getElementById(domId);
        const echartsInstance = echarts.init(domElement);
        const resizeChart = function () {
            const w = Math.min(780, window.innerWidth, window.innerHeight) - 30;
            domElement.style.width = w + 'px';
            domElement.style.height = w + 'px';
            echartsInstance.resize();
        };
        resizeChart();
        window.addEventListener('resize', resizeChart);

        const symbolSizeFunc = function (a) {
            return Math.round(Math.sqrt(a[2]) / 3) + 2;
        };
        const option = {
            title: {
                text: 'Three Body System',
                subtext: 'http://elasticdogs.com/three-body/index.html',
                left: 'center'
            },
            backgroundColor: new echarts.graphic.RadialGradient(0.3, 0.3, 0.8, [{
                offset: 0,
                color: '#f7f8fa'
            }, {
                offset: 1,
                color: '#cdd0d5'
            }]),
            toolbox: {
                feature: {
                    saveAsImage: {}
                }
            },
            polar: {},
            angleAxis: {
                type: 'value',
                startAngle: 0,
                silent: true,
                min: 0,
                max: 360
            },
            radiusAxis: {
                silent: true
            },
            series: [{
                coordinateSystem: 'polar',
                zlevel: 5,
                silent: true,
                type: 'effectScatter',
                data: [],
                symbolSize: symbolSizeFunc,
                itemStyle: {
                    normal: {
                        color: '#f07b3f',
                        shadowBlur: 10,
                        shadowColor: '#333'
                    }
                }
            }, {
                coordinateSystem: 'polar',
                name: '1',
                zlevel: 4,
                symbol: 'none',
                silent: true,
                type: 'line',
                data: [],
                lineStyle: {
                    color: '#00b8a9',
                    width: 2
                }
            }, {
                coordinateSystem: 'polar',
                name: '1',
                zlevel: 3,
                symbol: 'none',
                silent: true,
                type: 'line',
                lineStyle: {
                    color: '#f57170',
                    width: 2
                },
                data: []
            }, {
                coordinateSystem: 'polar',
                name: '1',
                zlevel: 2,
                symbol: 'none',
                silent: true,
                type: 'line',
                lineStyle: {
                    color: '#00e0ff',
                    width: 2
                },
                data: []
            }],
            animation: false
        };

        echartsInstance.setOption(option);

        return echartsInstance;
    };

    const echartsInstance = initChart('three-body-animation');

    window.updateThreeBodyFrame = function (vectors, traces) {
        echartsInstance.setOption({
            series: [{
                data: vectors
            }, {
                data: traces[0]
            }, {
                data: traces[1]
            }, {
                data: traces[2]
            }]
        });
    };

})(echarts);