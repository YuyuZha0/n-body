(function (odex) {

    const twoBodySystem = function (m1, m2) {
        return function (t, u) {
            const r = u[0] + u[4];
            const q = 1 / (r * r);
            const v = new Array(8);
            v[0] = u[2];
            v[1] = u[3];
            v[2] = u[0] * u[3] * u[3] - m2 * q;
            v[3] = -2 * u[2] * u[3] / u[0];
            v[4] = u[6];
            v[5] = u[7];
            v[6] = u[4] * u[7] * u[7] - m1 * q;
            v[7] = -2 * u[6] * u[7] / u[4];
            return v;
        };
    };

    const y0 = function (options) {
        const m1 = options.m1, m2 = options.m2, vec1 = options.u1;
        const massRatio = m1 / m2;
        const a = new Array(8);
        a[0] = vec1[0];
        a[1] = vec1[1];
        a[2] = vec1[2];
        a[3] = vec1[3];
        a[4] = massRatio * vec1[0];
        a[5] = vec1[1] + Math.PI;
        a[6] = massRatio * vec1[2];
        a[7] = vec1[3];
        return a;
    };

    const toPaintableVector = function (y, options) {
        const v1 = new Array(3), v2 = new Array(3);
        v1[0] = y[0];
        v1[1] = toStdDegree(y[1]);
        v1[2] = options.m1;
        v2[0] = y[4];
        v2[1] = toStdDegree(y[5]);
        v2[2] = options.m2;
        return [v1, v2];
    };

    window.readOptionsForPainting = function (options) {
        return toPaintableVector(y0(options), options);
    };

    const toStdDegree = function (x) {
        const d = x * 180 / Math.PI;
        return d - 360 * Math.floor(d / 360);
    };

    window.twoBodySimulator = function (options, renderer) {

        const defaultOptions = {
            m1: 1,
            m2: 1,
            u1: [1, 0, 0, 1],
            maxTraceKeep: 3000,
            timeDelta: 0.003
        };
        options = Object.assign(defaultOptions, options);

        if (typeof renderer !== 'function' || !renderer instanceof Function) {
            throw Error('renderer must be a function!');
        }

        const solver = new odex.Solver(8);
        solver.absoluteTolerance = solver.relativeTolerance = 1e-10;
        const f = twoBodySystem(options.m1, options.m2);
        let y = y0(options);
        let t = 0;

        const trace1 = [], trace2 = [];

        return function () {
            const currentY = solver.solve(f, t, y, t + options.timeDelta).y;
            const r1 = currentY[0], r2 = currentY[4];
            if (Number.isNaN(r1) || Number.isNaN(r2)) {
                console.info("terminated for collision, current time:{}", t);
                return;
            }

            const [vector1, vector2] = toPaintableVector(currentY, options);

            if (trace1.length > options.maxTraceKeep) {
                trace1.shift();
                trace2.shift();
            }
            trace1.push(vector1);
            trace2.push(vector2);
            y = currentY;
            t += options.timeDelta;
            renderer(vector1, vector2, trace1, trace2);
        }
    };

})(require('odex'));

(function (echarts) {

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
                text: 'Two Body System',
                subtext: 'http://elasticdogs.com/two-body/index.html',
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
            },
                {
                    coordinateSystem: 'polar',
                    name: '1',
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
                    symbol: 'none',
                    silent: true,
                    type: 'line',
                    lineStyle: {
                        color: '#f57170',
                        width: 2
                    },
                    data: []
                }],
            animation: false
        };

        echartsInstance.setOption(option);

        return echartsInstance;
    };

    const echartsInstance = initChart('two-body-animation');

    window.updateTwoBodyFrame = function (vector1, vector2, trace1, trace2) {
        echartsInstance.setOption({
            series: [{
                data: [vector1, vector2]
            }, {
                data: trace1
            }, {
                data: trace2
            }]
        })
        ;
    };

})(echarts);