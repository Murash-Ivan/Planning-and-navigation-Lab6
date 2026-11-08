using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace Lab_6
{
    public partial class Form1 : Form
    {
        //Переменные для UDP и потока
        private UdpClient udpClient;
        private Thread receiveThread;
        private bool isRunning = false;
        private int commandCounter = 0;

        //Глобальные координаты робота (из симулятора)
        private double currentX = 0.75;
        private double currentY = 0.75;
        private double currentTheta = 0; // Курс в радианах

        //Для отрисовки на Bitmap
        private Bitmap mapBitmap;
        private Graphics mapGraphics;
        private const double Scale = 35.0; // 35 пикселей - 1 метр

        //Углы дальномеров относительно курса
        private readonly double[] sensorAngles = {
            0, Math.PI / 4, Math.PI / 2, 3 * Math.PI / 4,
            Math.PI, -3 * Math.PI / 4, -Math.PI / 2, -Math.PI / 4 };

        //Переменные для глобального маршрута
        private List<PointF> routePoints = new List<PointF>(); //Список точек
        private int currentTargetIndex = 0;                    //Индекс текущей целевой точки
        private bool isAutoNavigating = false;                 //Флаг включения автодвижения

        private int frontDistance = 0;
        private int[] lastSensorDistances = new int[8]; //Последние данные со всех 8 дальномеров

        //Переменные для DWA
        private double currentVelocity = 0;      //Текущая продольная скорость v
        private double currentOmega = 0;         //Текущая угловая скорость w
        private const double RobotRadius = 0.3;  //Радиус робота м
        private const double TimeStep = 0.1;     //Шаг моделирования сек
        private const double SimTime = 2.0;      //Время моделирования траектории сек

        //Память для динамического препятствий
        private class SensedObstacle
        {
            public double X { get; set; }
            public double Y { get; set; }
            public DateTime LastSeen { get; set; }
        }
        private List<SensedObstacle> sensedObstacles = new List<SensedObstacle>();
        private const double ObstacleMemorySeconds = 10.0;  //Время запоминания препятствия
        private const double ObstacleMergeDistance = 0.7;  //Объединяю, если новое обнаружение ближе 70 см
        private const double ObstacleRadius = 0.25;  //Радиус динамического препятствия

        //Для стабильности аварийного движения
        private DateTime lastAvoidanceTime = DateTime.MinValue;
        private int lastAvoidanceDirection = 0;  //1 - влево, -1 - вправо

        //Для визуализации DWA
        private class TrajectoryInfo
        {
            public List<(double x, double y)> Points;
            public double Score;
            public double V;
            public double Omega;
            public bool IsCollision;
        }

        private List<TrajectoryInfo> allTrajectories = new List<TrajectoryInfo>();
        private TrajectoryInfo bestTrajectory = null;

        //Для отрисовки карты из файла
        private char[,] mapData = new char[40, 64];
        private bool isMapLoaded = false;

        public Form1()
        {
            InitializeComponent();
            this.KeyPreview = true;
            InitializeMap();
        }

        //Инициализация битмапа
        private void InitializeMap()
        {
            if (pbMap.Width > 0 && pbMap.Height > 0)
            {
                mapBitmap = new Bitmap(pbMap.Width, pbMap.Height);
                mapGraphics = Graphics.FromImage(mapBitmap);
                mapGraphics.Clear(Color.White);
                pbMap.Image = mapBitmap;
            }
        }

        //UDP: подключение и отключение
        private void StartUdpServer()
        {
            try
            {
                int localPort = int.Parse(txtLocalPort.Text);
                udpClient = new UdpClient(localPort);
                isRunning = true;

                receiveThread = new Thread(ReceiveData) { IsBackground = true };
                receiveThread.Start();

                lblStatus.Text = "Статус: Подключено";
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Ошибка подключения: {ex.Message}");
            }
        }

        private void StopUdpServer()
        {
            isRunning = false;
            udpClient?.Close();

            if (receiveThread != null && receiveThread.IsAlive)
            {
                receiveThread.Join(1000);
            }
            lblStatus.Text = "Статус: Отключено";
        }

        //UDP: прием данных
        private void ReceiveData()
        {
            IPEndPoint remoteEndPoint = new IPEndPoint(IPAddress.Any, 0);
            while (isRunning)
            {
                try
                {
                    byte[] data = udpClient.Receive(ref remoteEndPoint);
                    string message = Encoding.UTF8.GetString(data);

                    this.Invoke((MethodInvoker)delegate { UpdateUI(message); });
                }
                catch (Exception ex)
                {
                    if (isRunning) Console.WriteLine($"Ошибка приема: {ex.Message}");
                }
            }
        }

        //Обработка полученного сообщения
        private void UpdateUI(string message)
        {
            message = message.Trim();
            if (string.IsNullOrEmpty(message) || message == "0" || message.Contains("0 0 0 0 0")) return;

            if (message.Contains("{") && message.Contains("}"))
            {
                txtRawData.AppendText(message + Environment.NewLine);
                txtRawData.AppendText(new string('-', 50) + Environment.NewLine);

                txtRawData.SelectionStart = txtRawData.Text.Length;
                txtRawData.ScrollToCaret();

                if (txtRawData.Text.Length > 10000)
                {
                    int cutOff = txtRawData.Text.IndexOf(Environment.NewLine);
                    if (cutOff > 0) txtRawData.Text = txtRawData.Text.Substring(cutOff);
                }

                ParseAndDisplayData(message);
            }
        }

        //Парсинг JSON и обновление интерфейса
        private void ParseAndDisplayData(string jsonMessage)
        {
            try
            {
                string simXStr = GetJsonValue(jsonMessage, "long");
                if (simXStr == "0" || string.IsNullOrEmpty(simXStr)) simXStr = GetJsonValue(jsonMessage, "x");

                string simYStr = GetJsonValue(jsonMessage, "latt");
                if (simYStr == "0" || string.IsNullOrEmpty(simYStr)) simYStr = GetJsonValue(jsonMessage, "y");

                string simTStr = GetJsonValue(jsonMessage, "course");
                if (simTStr == "0" || string.IsNullOrEmpty(simTStr)) simTStr = GetJsonValue(jsonMessage, "t");

                if (double.TryParse(simXStr, NumberStyles.Any, CultureInfo.InvariantCulture, out double gx)) currentX = gx;
                if (double.TryParse(simYStr, NumberStyles.Any, CultureInfo.InvariantCulture, out double gy)) currentY = gy;
                if (double.TryParse(simTStr, NumberStyles.Any, CultureInfo.InvariantCulture, out double gt)) currentTheta = gt * Math.PI / 180.0;

                txtX.Text = currentX.ToString("F2", CultureInfo.InvariantCulture);
                txtY.Text = currentY.ToString("F2", CultureInfo.InvariantCulture);
                txtTheta.Text = (currentTheta * 180.0 / Math.PI).ToString("F1", CultureInfo.InvariantCulture);

                txtX.Text = txtX.Text;
                txtY.Text = txtY.Text;
                txtTheta.Text = txtTheta.Text;

                txtLeftEncoder.Text = GetJsonValue(jsonMessage, "le");
                txtRightEncoder.Text = GetJsonValue(jsonMessage, "re");

                txtCode.Text = GetJsonValue(jsonMessage, "c");
                txtBumper.Text = GetJsonValue(jsonMessage, "b");

                int[] d = new int[8];
                for (int i = 0; i < 8; i++)
                {
                    d[i] = int.TryParse(GetJsonValue(jsonMessage, $"d{i}"), out int val) ? val : 0;
                    var ctrl = this.Controls.Find($"txtD{i}", true);
                    if (ctrl.Length > 0) ctrl[0].Text = d[i].ToString();
                }

                frontDistance = d[0];
                lastSensorDistances = d;

                UpdateDynamicObstacleMemory();

                for (int i = 0; i < 5; i++)
                {
                    var ctrl = this.Controls.Find($"txtL{i}", true);
                    if (ctrl.Length > 0) ctrl[0].Text = GetJsonValue(jsonMessage, $"l{i}");
                }

                DrawMap(d);
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Ошибка парсинга: {ex.Message}");
            }
        }

        //Извлечение значений из JSON строки
        private string GetJsonValue(string json, string key)
        {
            string searchPattern = $"\"{key}\":";
            int keyIndex = json.IndexOf(searchPattern);
            if (keyIndex == -1) return "0";

            int valueStart = keyIndex + searchPattern.Length;
            while (valueStart < json.Length && (json[valueStart] == ' ' || json[valueStart] == '"')) valueStart++;

            int valueEnd = valueStart;
            while (valueEnd < json.Length && json[valueEnd] != ',' && json[valueEnd] != '}' && json[valueEnd] != '"') valueEnd++;

            if (valueStart >= valueEnd) return "0";
            string value = json.Substring(valueStart, valueEnd - valueStart).Trim();
            return string.IsNullOrEmpty(value) ? "0" : value;
        }

        //Визуализация на битмапе
        private void DrawMap(int[] d)
        {
            if (mapGraphics == null) return;

            mapGraphics.Clear(Color.White);

            //Отрисовка стен карты
            if (isMapLoaded)
            {
                double cellSizeMeters = 0.25;
                double cellSizePixels = cellSizeMeters * Scale;

                float yOffset = pbMap.Height - (float)(40 * cellSizePixels);
                if (yOffset < 0) yOffset = 0;

                for (int r = 0; r < 40; r++)
                {
                    for (int c = 0; c < 64; c++)
                    {
                        if (mapData[r, c] == '#')
                        {
                            float x = (float)(c * cellSizePixels);

                            float y = yOffset + (float)(r * cellSizePixels);

                            mapGraphics.FillRectangle(Brushes.Black, x, y,
                                (float)cellSizePixels + 1, (float)cellSizePixels + 1);
                        }
                    }
                }
            }

            //Отрисовка маршрута
            if (routePoints.Count > 0)
            {
                using (Pen routePen = new Pen(Color.Green, 2))
                {
                    for (int i = 0; i < routePoints.Count; i++)
                    {
                        int x1 = (int)(routePoints[i].X * Scale);
                        int y1 = pbMap.Height - (int)(routePoints[i].Y * Scale);

                        int nextIdx = (i + 1) % routePoints.Count;
                        int x2 = (int)(routePoints[nextIdx].X * Scale);
                        int y2 = pbMap.Height - (int)(routePoints[nextIdx].Y * Scale);

                        mapGraphics.DrawLine(routePen, x1, y1, x2, y2);
                        mapGraphics.FillEllipse(Brushes.Green, x1 - 3, y1 - 3, 6, 6);
                    }
                }

                PointF currTarget = routePoints[currentTargetIndex];
                int tx = (int)(currTarget.X * Scale);
                int ty = pbMap.Height - (int)(currTarget.Y * Scale);
                int radiusPx = (int)((double)numArrivalRadius.Value * Scale);

                using (Pen radiusPen = new Pen(Color.Red, 1))
                {
                    radiusPen.DashStyle = System.Drawing.Drawing2D.DashStyle.Dash;
                    mapGraphics.DrawEllipse(radiusPen, tx - radiusPx, ty - radiusPx, radiusPx * 2, radiusPx * 2);
                }
                mapGraphics.FillEllipse(Brushes.Red, tx - 5, ty - 5, 10, 10);
            }

            //Отрисовка робота и дальномеров
            int screenX = (int)(currentX * Scale);
            int screenY = pbMap.Height - (int)(currentY * Scale);

            float robotRadius = (float)(0.3 * Scale);
            using (SolidBrush robotBrush = new SolidBrush(Color.FromArgb(150, 255, 165, 0)))
            using (Pen robotPen = new Pen(Color.DarkOrange, 2))
            {
                mapGraphics.FillEllipse(robotBrush, screenX - robotRadius, screenY - robotRadius, robotRadius * 2, robotRadius * 2);
                mapGraphics.DrawEllipse(robotPen, screenX - robotRadius, screenY - robotRadius, robotRadius * 2, robotRadius * 2);
            }

            float headingX = screenX + (float)(robotRadius * Math.Sin(currentTheta));
            float headingY = screenY - (float)(robotRadius * Math.Cos(currentTheta));

            using (Pen headingPen = new Pen(Color.Red, 3))
            {
                mapGraphics.DrawLine(headingPen, screenX, screenY, headingX, headingY);
            }

            using (Pen rayPen = new Pen(Color.LightGray, 1))
            using (SolidBrush obsBrush = new SolidBrush(Color.Blue))
            {
                float robotRadiusM = 0.3f;

                for (int i = 0; i < 8; i++)
                {
                    if (d[i] > 0)
                    {
                        double distM = d[i] / 100.0;
                        double globalDirection = currentTheta + sensorAngles[i];

                        double sensorX = currentX + robotRadiusM * Math.Sin(globalDirection);
                        double sensorY = currentY + robotRadiusM * Math.Cos(globalDirection);

                        double obsX = sensorX + distM * Math.Sin(globalDirection);
                        double obsY = sensorY + distM * Math.Cos(globalDirection);

                        int sensorScreenX = (int)(sensorX * Scale);
                        int sensorScreenY = pbMap.Height - (int)(sensorY * Scale);

                        int obsScreenX = (int)(obsX * Scale);
                        int obsScreenY = pbMap.Height - (int)(obsY * Scale);

                        mapGraphics.DrawLine(rayPen, sensorScreenX, sensorScreenY, obsScreenX, obsScreenY);

                        if (d[i] < 300)
                        {
                            mapGraphics.FillEllipse(obsBrush, obsScreenX - 3, obsScreenY - 3, 6, 6);
                        }
                    }
                }
            }

            //Визуализация DWA (если включен)
            if (chkUseDWA.Checked && isAutoNavigating)
            {
                DrawDWAVisualization();
            }

            pbMap.Refresh();
        }

        //Отправка команд управления
        private void SendCommand(int f, int b)
        {
            if (!isRunning || udpClient == null) return;
            try
            {
                commandCounter++;
                string json = $"{{\"N\":{commandCounter},\"M\":0,\"F\":{f},\"B\":{b},\"T\":0}}\n";

                string remoteIp = txtRemoteIP.Text;
                int remotePort = int.Parse(txtRemotePort.Text);

                byte[] data = Encoding.UTF8.GetBytes(json);
                udpClient.Send(data, data.Length, remoteIp, remotePort);
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Ошибка отправки: {ex.Message}");
            }
        }

        private void btnConnect_Click(object sender, EventArgs e)
        {
            if (!isRunning)
            {
                StartUdpServer();
                btnConnect.Text = "Отключиться";
            }
            else
            {
                StopUdpServer();
                btnConnect.Text = "Подключиться";
            }
        }

        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            if (isRunning) StopUdpServer();
            mapGraphics?.Dispose();
            mapBitmap?.Dispose();
        }

        private void btnSendManual_Click(object sender, EventArgs e)
        {
            int f = int.TryParse(txtForce.Text, out int resF) ? resF : 0;
            int b = int.TryParse(txtBalance.Text, out int resB) ? resB : 0;
            SendCommand(f, b);
        }

        private void btnStop_Click(object sender, EventArgs e)
        {
            tbForce.Value = 0; tbBalance.Value = 0;
            txtForce.Text = "0"; txtBalance.Text = "0";
            SendCommand(0, 0);
        }

        private void tbForce_Scroll(object sender, EventArgs e)
        {
            txtForce.Text = tbForce.Value.ToString();
        }

        private void tbBalance_Scroll(object sender, EventArgs e)
        {
            txtBalance.Text = tbBalance.Value.ToString();
        }

        private void btnForward_Click(object sender, EventArgs e)
        {
            tbForce.Value = 50; tbBalance.Value = 0; txtForce.Text = "50"; txtBalance.Text = "0"; SendCommand(50, 0);
        }

        private void btnBackward_Click(object sender, EventArgs e)
        {
            tbForce.Value = -50; tbBalance.Value = 0; txtForce.Text = "-50"; txtBalance.Text = "0"; SendCommand(-50, 0);
        }

        private void btnLeft_Click(object sender, EventArgs e)
        {
            tbForce.Value = 0; tbBalance.Value = 30; txtForce.Text = "0"; txtBalance.Text = "30"; SendCommand(0, 30);
        }

        private void btnRight_Click(object sender, EventArgs e)
        {
            tbForce.Value = 0; tbBalance.Value = -30; txtForce.Text = "0"; txtBalance.Text = "-30"; SendCommand(0, -30);
        }

        private void Form1_KeyDown(object sender, KeyEventArgs e)
        {
            switch (e.KeyCode)
            {
                case Keys.Up: case Keys.W: btnForward_Click(sender, e); break;
                case Keys.Down: case Keys.S: btnBackward_Click(sender, e); break;
                case Keys.Left: case Keys.A: btnLeft_Click(sender, e); break;
                case Keys.Right: case Keys.D: btnRight_Click(sender, e); break;
                case Keys.Space: btnStop_Click(sender, e); break;
            }
        }

        private void btnLoadRoute_Click(object sender, EventArgs e)
        {
            if (ofd.ShowDialog() == DialogResult.OK)
            {
                routePoints.Clear();
                try
                {
                    var lines = File.ReadAllLines(ofd.FileName);
                    foreach (var line in lines)
                    {
                        if (string.IsNullOrWhiteSpace(line)) continue;

                        var parts = line.Split(new char[] { ',', ';', ' ' }, StringSplitOptions.RemoveEmptyEntries);
                        if (parts.Length >= 2)
                        {
                            if (double.TryParse(parts[0], NumberStyles.Any, CultureInfo.InvariantCulture, out double x) &&
                                double.TryParse(parts[1], NumberStyles.Any, CultureInfo.InvariantCulture, out double y))
                            {
                                routePoints.Add(new PointF((float)x, (float)y));
                            }
                        }
                    }

                    if (routePoints.Count > 0)
                    {
                        currentTargetIndex = 0;
                        lblRouteInfo.Text = $"Загружено: {routePoints.Count} точек";
                        lblTargetInfo.Text = $"Цель: 1 / {routePoints.Count}";
                        btnToggleAuto.Enabled = true;
                        pbMap.Refresh();
                    }
                    else
                    {
                        lblRouteInfo.Text = "Ошибка: файл пуст или неверный формат";
                    }
                }
                catch (Exception ex)
                {
                    MessageBox.Show($"Ошибка чтения файла: {ex.Message}");
                }
            }
        }

        private void btnToggleAuto_Click(object sender, EventArgs e)
        {
            isAutoNavigating = !isAutoNavigating;

            if (isAutoNavigating)
            {
                tmrNavigation.Enabled = true;
                btnToggleAuto.Text = "Стоп авто";
            }
            else
            {
                tmrNavigation.Enabled = false;
                SendCommand(0, 0);
                btnToggleAuto.Text = "Старт авто";
            }
        }

        private void tmrNavigation_Tick(object sender, EventArgs e)
        {
            if (!isAutoNavigating || routePoints.Count == 0) return;

            PointF target = routePoints[currentTargetIndex];

            //Считаю расстояние до цели
            double dx = target.X - currentX;
            double dy = target.Y - currentY;
            double dist = Math.Sqrt(dx * dx + dy * dy);

            //Проверка достижения цели
            double arrivalRadius = (double)numArrivalRadius.Value;
            if (dist <= arrivalRadius)
            {
                currentTargetIndex = (currentTargetIndex + 1) % routePoints.Count;
                lblTargetInfo.Text = $"Цель: {currentTargetIndex + 1} / {routePoints.Count}";
                pbMap.Refresh();

                //Сбрасываю скорости при достижении цели
                currentVelocity = 0;
                currentOmega = 0;
                SendCommand(0, 0);
                return;
            }

            int F = 0;
            int B = 0;

            //DWA или П-регулятор
            if (chkUseDWA.Checked)
            {
                // Логика DWA
                var (bestV, bestOmega) = CalculateDWA(
                    currentX, currentY, currentTheta,
                    target.X, target.Y,
                    lastSensorDistances);

                currentVelocity = bestV;
                currentOmega = bestOmega;

                double maxV_mps = (double)numMaxVelocity.Value / 100.0;
                if (maxV_mps > 0)
                {
                    F = (int)((currentVelocity / maxV_mps) * 100.0);
                }
                else
                {
                    F = 0;
                }
                F = Math.Max(-100, Math.Min(100, F));

                double maxOmega = (double)numMaxOmega.Value;
                B = (int)(-currentOmega / maxOmega * 100);
                B = Math.Max(-100, Math.Min(100, B));

                lblRouteInfo.Text = $"[DWA] v={currentVelocity:F2} м/с, w={currentOmega:F2} рад/с";
            }
            else
            {
                //Логика п-регулятора
                double targetAngle = Math.Atan2(dx, dy);
                double errorAngle = targetAngle - currentTheta;

                while (errorAngle > Math.PI) errorAngle -= 2 * Math.PI;
                while (errorAngle < -Math.PI) errorAngle += 2 * Math.PI;

                double errorDeg = errorAngle * 180.0 / Math.PI;

                if (frontDistance > 0 && frontDistance < 50)
                {
                    F = -30;
                    B = 0;
                    lblRouteInfo.Text = $"[П-рег] СТЕНА ({frontDistance} см)! Отъезд.";
                }
                else
                {
                    if (Math.Abs(errorDeg) < 5.0)
                    {
                        B = 0;
                    }
                    else
                    {
                        B = (int)(-errorDeg * 1.5);
                        B = Math.Max(-100, Math.Min(100, B));
                    }

                    if (Math.Abs(errorDeg) > 60.0)
                    {
                        F = 0;
                        lblRouteInfo.Text = $"[П-рег] Поворот на месте (ошибка {errorDeg:F1})";
                    }
                    else
                    {
                        if (dist > 2.0) F = 60;
                        else if (dist > 1.0) F = 30;
                        else F = 15;

                        lblRouteInfo.Text = $"[П-рег] Движение (Дист: {dist:F2}м, Угол: {errorDeg:F1})";
                    }
                }
            }

            //Отправляю команду
            SendCommand(F, B);

            //Обновляю информацию о цели на форме
            lblTargetInfo.Text = $"Цель: {currentTargetIndex + 1} | Дист: {dist:F2}м | F={F} B={B}";
        }

        // Алгоритм DWA
        private (double bestV, double bestOmega) CalculateDWA(
                double robotX, double robotY, double robotTheta,
                double targetX, double targetY,
                int[] sensorDistances)
        {
            //Очищаю списки для новой визуализации
            allTrajectories.Clear();
            bestTrajectory = null;

            //Получаю параметры с формы
            double maxV = (double)numMaxVelocity.Value / 100.0;
            double maxOmega = (double)numMaxOmega.Value;
            double alpha = (double)numAlpha.Value;
            double beta = (double)numBeta.Value;
            double gamma = (double)numGamma.Value;
            double maxAccelV = (double)numMaxAccelV.Value;
            double maxAccelOmega = (double)numMaxAccelOmega.Value;
            double stepV = (double)numStepV.Value;
            double stepOmega = (double)numStepOmega.Value;

            //Вычисляю динамическое окно
            double vMinDyn = Math.Max(-maxV * 0.3, currentVelocity - maxAccelV * TimeStep);
            double vMaxDyn = Math.Min(maxV, currentVelocity + maxAccelV * TimeStep);

            double omegaMinDyn = Math.Max(-maxOmega, currentOmega - maxAccelOmega * TimeStep);
            double omegaMaxDyn = Math.Min(maxOmega, currentOmega + maxAccelOmega * TimeStep);

            if (vMaxDyn < vMinDyn + stepV) vMaxDyn = vMinDyn + stepV;
            if (omegaMaxDyn < omegaMinDyn + stepOmega) omegaMaxDyn = omegaMinDyn + stepOmega;

            double bestScore = -1000.0;
            double bestV = 0.0;
            double bestOmega = 0.0;
            bool foundSafeTrajectory = false;

            for (double v = vMinDyn; v <= vMaxDyn; v += stepV)
            {
                for (double omega = omegaMinDyn; omega <= omegaMaxDyn; omega += stepOmega)
                {
                    //Моделирую траекторию
                    List<(double x, double y)> trajectory = SimulateTrajectory(robotX, robotY, robotTheta, v, omega, SimTime);

                    //Проверяю коллизии
                    bool collision = CheckCollision(trajectory, sensorDistances, robotX, robotY, robotTheta);

                    //Сохраняю траекторию для визуализации
                    TrajectoryInfo trajInfo = new TrajectoryInfo
                    {
                        Points = trajectory,
                        V = v,
                        Omega = omega,
                        IsCollision = collision,
                        Score = 0
                    };

                    if (collision)
                    {
                        allTrajectories.Add(trajInfo);
                        continue;
                    }

                    foundSafeTrajectory = true;

                    //Оцениваю траекторию
                    //Вычисляю, куда робот будет смотреть в конце этой траектории
                    double finalTheta = robotTheta + omega * SimTime;
                    double heading = CalculateHeading(trajectory.Last(), targetX, targetY, finalTheta);
                    double clearance = CalculateClearance(trajectory, sensorDistances, robotX, robotY, robotTheta);
                    double velocityScore = v > 0 ? (v / maxV) : (v / maxV) * 0.5;

                    double score = (alpha * heading) + (beta * clearance) + (gamma * velocityScore);
                    trajInfo.Score = score;

                    //Запоминаю лучшую
                    if (score > bestScore)
                    {
                        bestScore = score;
                        bestV = v;
                        bestOmega = omega;
                        bestTrajectory = trajInfo;
                    }

                    allTrajectories.Add(trajInfo);
                }
            }

            // Если нет безопасной траектории или лучшая траектория слишком медленная (< 5 см/с), то это аварийный режим
            if (!foundSafeTrajectory || bestV < 0.05)
            {
                if (bestTrajectory != null && bestV >= 0.05)
                {
                    return (bestV, bestOmega);
                }


                double leftDist = GetDistanceToWall(robotX, robotY, robotTheta + Math.PI / 2);
                double rightDist = GetDistanceToWall(robotX, robotY, robotTheta - Math.PI / 2);

                //Проверяю, не в режиме ли объезда (последние 1.5 сек)
                bool isInAvoidance = (DateTime.Now - lastAvoidanceTime).TotalSeconds < 1.5;

                double turnDir;

                //Если уже в режиме объезда, сохраняю направление
                if (isInAvoidance && lastAvoidanceDirection != 0)
                {
                    turnDir = lastAvoidanceDirection * 1.5;
                }
                else
                {
                    //Новый маневр, выбираю направление
                    turnDir = (leftDist > rightDist) ? 1.5 : -1.5;
                    lastAvoidanceDirection = (leftDist > rightDist) ? 1 : -1;
                    lastAvoidanceTime = DateTime.Now;
                }

                //Поворот
                return (0.00, turnDir);
            }

            return (bestV, bestOmega);
        }

        //Моделирование траектории робота на время simTime
        private List<(double x, double y)> SimulateTrajectory(
            double startX, double startY, double startTheta,
            double v, double omega, double simTime)
        {
            List<(double x, double y)> trajectory = new List<(double x, double y)>();
            double x = startX;
            double y = startY;
            double theta = startTheta;

            for (double t = 0; t < simTime; t += TimeStep)
            {
                x += v * Math.Sin(theta) * TimeStep;
                y += v * Math.Cos(theta) * TimeStep;
                theta += omega * TimeStep;
                trajectory.Add((x, y));
            }

            return trajectory;
        }

        // Проверяет, есть ли коллизия траектории с препятствиями
        private bool CheckCollision(
            List<(double x, double y)> trajectory,
            int[] sensorDistances,
            double robotX, double robotY, double robotTheta)
        {
            //Запас 5 см
            //Радиус проверки = 0.3 (робот) + 0.05 (запас) = 0.35 м.
            double safetyMargin = 0.02;
            double checkRadius = RobotRadius + safetyMargin;

            foreach (var point in trajectory)
            {
                //Проверка стен карты
                //Проверяю не точку, а область вокруг неё размером с робота + запас
                if (IsNearStaticWall(point.x, point.y, checkRadius))
                {
                    return true; // Траектория задевает стену габаритами робота
                }

                //Проверка динамического препятствия
                foreach (var memObs in sensedObstacles)
                {
                    double dx = point.x - memObs.X;
                    double dy = point.y - memObs.Y;
                    double dist = Math.Sqrt(dx * dx + dy * dy);

                    //Радиус робота + радиус препятствия + запас 10 см
                    if (dist < RobotRadius + ObstacleRadius + 0.02)
                    {
                        return true;
                    }
                }
            }

            return false;
        }

        //Вычисляет компонент heading (направление к цели)
        //finalTheta - это курс робота в конце траектории, а не в начале
        private double CalculateHeading(
            (double x, double y) lastPoint,
            double targetX, double targetY,
            double finalTheta)
        {
            //Считаю направление от конца траектории к цели
            double dx = targetX - lastPoint.x;
            double dy = targetY - lastPoint.y;
            double targetAngle = Math.Atan2(dx, dy);

            //Считаю ошибку между финальным курсом робота и направлением на цель
            double error = targetAngle - finalTheta;

            //Нормализация угла в диапазон [-PI, PI]
            while (error > Math.PI) error -= 2 * Math.PI;
            while (error < -Math.PI) error += 2 * Math.PI;

            //Нормализую в диапазон [0, 1]
            //1.0 - робот смотрит прямо на цель
            //0.0 - робот смотрит в противоположную сторону
            return 1.0 - Math.Abs(error) / Math.PI;
        }

        // Вычисляет компонент clearance (расстояние до ближайшего препятствия)
        // Учитывает:
        //   1) текущие показания дальномеров (поверхность обнаруженных объектов)
        //   2) память о динамических препятствиях (sensedObstacles), которые робот видел ранее, но сейчас может не видеть
        private double CalculateClearance(
            List<(double x, double y)> trajectory,
            int[] sensorDistances,
            double robotX, double robotY, double robotTheta)
        {
            double minDist = double.MaxValue;

            //Вклад текущих показаний дальномеров (расстояние от точки траектории до обнаруженной поверхности)
            foreach (var point in trajectory)
            {
                for (int i = 0; i < 8; i++)
                {
                    if (sensorDistances[i] > 0)
                    {
                        double distM = sensorDistances[i] / 100.0;
                        double globalDirection = robotTheta + sensorAngles[i];

                        double sensorX = robotX + RobotRadius * Math.Sin(globalDirection);
                        double sensorY = robotY + RobotRadius * Math.Cos(globalDirection);
                        double obsX = sensorX + distM * Math.Sin(globalDirection);
                        double obsY = sensorY + distM * Math.Cos(globalDirection);

                        double dx = point.x - obsX;
                        double dy = point.y - obsY;
                        double dist = Math.Sqrt(dx * dx + dy * dy);

                        if (dist < minDist)
                            minDist = dist;
                    }
                }
            }

            //Вклад памяти о динамических препятствиях
            //Расстояние считаю до края препятствия, чтобы согласовать с показаниями дальномеров, которые тоже фиксируют поверхность объекта, а не его центр
            DateTime now = DateTime.Now;
            foreach (var point in trajectory)
            {
                foreach (var memObs in sensedObstacles)
                {
                    double dx = point.x - memObs.X;
                    double dy = point.y - memObs.Y;
                    double distToCenter = Math.Sqrt(dx * dx + dy * dy);

                    double distToSurface = Math.Max(0.0, distToCenter - ObstacleRadius);

                    double effectiveDist = distToSurface;

                    //Штраф, если ближе 40 см к препятствию
                    if (distToSurface < 0.40)
                    {
                        effectiveDist = Math.Min(effectiveDist, distToSurface * 0.3);
                    }

                    if (effectiveDist < minDist)
                        minDist = effectiveDist;
                }
            }

            //Если препятствий нет максимальная безопасность
            if (minDist == double.MaxValue)
                return 1.0;
            return Math.Min(minDist / 5.0, 1.0);
        }

        //Визуализация работы алгоритма DWA
        private void DrawDWAVisualization()
        {
            if (allTrajectories.Count == 0) return;

            //Рисую все траектории, полупрозрачные серые линии
            using (Pen trajectoryPen = new Pen(Color.FromArgb(30, 150, 150, 150), 1))
            {
                foreach (var traj in allTrajectories)
                {
                    if (traj.IsCollision) continue; //Коллизии рисую отдельно

                    for (int i = 0; i < traj.Points.Count - 1; i++)
                    {
                        int x1 = (int)(traj.Points[i].x * Scale);
                        int y1 = pbMap.Height - (int)(traj.Points[i].y * Scale);
                        int x2 = (int)(traj.Points[i + 1].x * Scale);
                        int y2 = pbMap.Height - (int)(traj.Points[i + 1].y * Scale);

                        mapGraphics.DrawLine(trajectoryPen, x1, y1, x2, y2);
                    }
                }
            }

            //Рисую траектории с коллизиями, красные пунктирные
            using (Pen collisionPen = new Pen(Color.FromArgb(50, 255, 0, 0), 1))
            {
                collisionPen.DashStyle = System.Drawing.Drawing2D.DashStyle.Dash;
                foreach (var traj in allTrajectories)
                {
                    if (!traj.IsCollision) continue;

                    for (int i = 0; i < traj.Points.Count - 1; i++)
                    {
                        int x1 = (int)(traj.Points[i].x * Scale);
                        int y1 = pbMap.Height - (int)(traj.Points[i].y * Scale);
                        int x2 = (int)(traj.Points[i + 1].x * Scale);
                        int y2 = pbMap.Height - (int)(traj.Points[i + 1].y * Scale);

                        mapGraphics.DrawLine(collisionPen, x1, y1, x2, y2);
                    }
                }
            }

            //Рисую тепловую карту, точки вдоль траекторий, окрашенные по score
            //Нахожу минимальный и максимальный score для нормализации
            double minScore = double.MaxValue;
            double maxScore = double.MinValue;
            foreach (var traj in allTrajectories)
            {
                if (!traj.IsCollision && traj.Score > 0)
                {
                    if (traj.Score < minScore) minScore = traj.Score;
                    if (traj.Score > maxScore) maxScore = traj.Score;
                }
            }

            if (maxScore > minScore)
            {
                foreach (var traj in allTrajectories)
                {
                    if (traj.IsCollision) continue;

                    //Нормализую score в диапазон [0, 1]
                    double normalizedScore = (traj.Score - minScore) / (maxScore - minScore);

                    //Цвет от красного (плохо) к зеленому (хорошо)
                    int red = (int)(255 * (1 - normalizedScore));
                    int green = (int)(255 * normalizedScore);
                    Color pointColor = Color.FromArgb(150, red, green, 0);

                    using (Brush pointBrush = new SolidBrush(pointColor))
                    {
                        //Рисую точки каждые 3 точки траектории для производительности
                        for (int i = 0; i < traj.Points.Count; i += 3)
                        {
                            int px = (int)(traj.Points[i].x * Scale);
                            int py = pbMap.Height - (int)(traj.Points[i].y * Scale);
                            mapGraphics.FillEllipse(pointBrush, px - 2, py - 2, 4, 4);
                        }
                    }
                }
            }

            //Рисую лучшую траекторию, ярко-зелёная линия
            if (bestTrajectory != null)
            {
                using (Pen bestPen = new Pen(Color.FromArgb(255, 0, 255, 0), 1))
                {
                    for (int i = 0; i < bestTrajectory.Points.Count - 1; i++)
                    {
                        int x1 = (int)(bestTrajectory.Points[i].x * Scale);
                        int y1 = pbMap.Height - (int)(bestTrajectory.Points[i].y * Scale);
                        int x2 = (int)(bestTrajectory.Points[i + 1].x * Scale);
                        int y2 = pbMap.Height - (int)(bestTrajectory.Points[i + 1].y * Scale);

                        mapGraphics.DrawLine(bestPen, x1, y1, x2, y2);
                    }

                    //Рисую маленькую точку в конце лучшей траектории
                    int endX = (int)(bestTrajectory.Points.Last().x * Scale);
                    int endY = pbMap.Height - (int)(bestTrajectory.Points.Last().y * Scale);
                    mapGraphics.FillEllipse(Brushes.Lime, endX - 3, endY - 3, 6, 6);
                }
            }

            //Рисуею препятствия, обнаруженные дальномерами
            using (SolidBrush obstacleBrush = new SolidBrush(Color.FromArgb(200, 255, 100, 0)))
            {
                foreach (var memObs in sensedObstacles)
                {
                    int obsScreenX = (int)(memObs.X * Scale);
                    int obsScreenY = pbMap.Height - (int)(memObs.Y * Scale);

                    //Рисую оранжевый кружок только для запомненных динамических препятствий
                    mapGraphics.FillEllipse(obstacleBrush, obsScreenX - 8, obsScreenY - 8, 16, 16);

                    //Рисую время жизни воспоминания
                    double age = (DateTime.Now - memObs.LastSeen).TotalSeconds;
                    using (Font ageFont = new Font("Arial", 6))
                    {
                        mapGraphics.DrawString($"{age:F1}s", ageFont, Brushes.Black,
                            obsScreenX - 10, obsScreenY - 18);
                    }
                }
            }

            //Рисую легенду
            using (Font legendFont = new Font("Arial", 9, FontStyle.Bold))
            using (Brush textBrush = new SolidBrush(Color.Black))
            {
                int legendY = 5;
                int legendHeight = 25;

                //Фон легенды
                using (Brush bgBrush = new SolidBrush(Color.FromArgb(220, 240, 240, 240)))
                using (Pen borderPen = new Pen(Color.Gray, 1))
                {
                    //Рисую фон на всю ширину карты
                    mapGraphics.FillRectangle(bgBrush, 0, legendY, pbMap.Width, legendHeight);
                    mapGraphics.DrawRectangle(borderPen, 0, legendY, pbMap.Width - 1, legendHeight);
                }

                // Равномерно распределяю 5 элементов по ширине карты
                int itemWidth = pbMap.Width / 5;

                //Лучшая траектория
                int startX = 10;
                mapGraphics.DrawLine(new Pen(Color.Lime, 3), startX, legendY + 12, startX + 20, legendY + 12);
                mapGraphics.DrawString("Лучшая", legendFont, textBrush, startX + 25, legendY + 5);

                //Веер траекторий
                startX = itemWidth * 1 + 10;
                mapGraphics.DrawLine(new Pen(Color.FromArgb(150, 200, 200, 200), 2), startX, legendY + 12, startX + 20, legendY + 12);
                mapGraphics.DrawString("Веер", legendFont, textBrush, startX + 25, legendY + 5);

                //Коллизии
                startX = itemWidth * 2 + 10;
                mapGraphics.DrawLine(new Pen(Color.FromArgb(200, 255, 0, 0), 2), startX, legendY + 12, startX + 20, legendY + 12);
                mapGraphics.DrawString("Коллизии", legendFont, textBrush, startX + 25, legendY + 5);

                //Препятствия
                startX = itemWidth * 3 + 10;
                mapGraphics.FillEllipse(new SolidBrush(Color.FromArgb(200, 255, 140, 0)), startX + 5, legendY + 7, 10, 10);
                mapGraphics.DrawString("Препятствия", legendFont, textBrush, startX + 20, legendY + 5);

                //Score (Тепловая карта)
                startX = itemWidth * 4 + 10;
                mapGraphics.FillRectangle(new SolidBrush(Color.FromArgb(200, 180, 180, 0)), startX + 5, legendY + 7, 10, 10);
                mapGraphics.DrawString("Score", legendFont, textBrush, startX + 20, legendY + 5);
            }
        }

        private void btnLoadMap_Click(object sender, EventArgs e)
        {
            OpenFileDialog ofd1 = new OpenFileDialog();
            ofd1.Filter = "Текстовые файлы карты|*.txt";
            ofd1.Title = "Выберите файл";

            if (ofd1.ShowDialog() == DialogResult.OK)
            {
                try
                {
                    string[] lines = File.ReadAllLines(ofd1.FileName);
                    if (lines.Length >= 40)
                    {
                        for (int r = 0; r < 40; r++)
                        {
                            string line = lines[r].Length >= 64 ? lines[r].Substring(0, 64) : lines[r].PadRight(64, '.');
                            for (int c = 0; c < 64; c++)
                            {
                                mapData[r, c] = line[c];
                            }
                        }
                        isMapLoaded = true;
                        pbMap.Refresh();
                        lblStatus.Text = "Статус: Карта загружена";
                    }
                    else
                    {
                        MessageBox.Show("Файл карты должен содержать минимум 40 строк!");
                    }
                }
                catch (Exception ex)
                {
                    MessageBox.Show("Ошибка загрузки карты: " + ex.Message);
                }
            }
        }

        //Проверяет есть ли статическая стена вблизи указанной точки
        private bool IsNearStaticWall(double x, double y, double tolerance = 0.15)
        {
            if (!isMapLoaded)
                return false;

            double cellWidth = 0.25;
            double cellHeight = 0.25;
            int mapColumns = 64;
            int mapRows = 40;
            double fieldHeight = 10.0;

            int minCol = Math.Max(0, (int)((x - tolerance) / cellWidth));
            int maxCol = Math.Min(mapColumns - 1, (int)((x + tolerance) / cellWidth));

            int minRow = Math.Max(0, mapRows - 1 - (int)((y + tolerance) / cellHeight));
            int maxRow = Math.Min(mapRows - 1, mapRows - 1 - (int)((y - tolerance) / cellHeight));

            for (int row = minRow; row <= maxRow; row++)
            {
                for (int col = minCol; col <= maxCol; col++)
                {
                    if (mapData[row, col] != '#')
                        continue;

                    double cellLeft = col * cellWidth;
                    double cellRight = cellLeft + cellWidth;
                    double cellBottom = fieldHeight - (row + 1) * cellHeight;
                    double cellTop = cellBottom + cellHeight;

                    double nearestX = Math.Max(cellLeft, Math.Min(x, cellRight));
                    double nearestY = Math.Max(cellBottom, Math.Min(y, cellTop));

                    double dx = x - nearestX;
                    double dy = y - nearestY;
                    double distance = Math.Sqrt(dx * dx + dy * dy);

                    if (distance <= tolerance)
                        return true;
                }
            }
            return false;
        }

        private void UpdateDynamicObstacleMemory()
        {
            DateTime now = DateTime.Now;

            //Удаляю старые воспоминания
            sensedObstacles.RemoveAll(o => (now - o.LastSeen).TotalSeconds > ObstacleMemorySeconds);

            //Обрабатываю текущие показания датчиков
            for (int i = 0; i < 8; i++)
            {
                double distM = lastSensorDistances[i] / 100.0;

                //Учитываю только реальные объекты на расстоянии от 1 см до 2.5 метров
                if (distM > 0.1 && distM < 2.5)
                {
                    double globalDirection = currentTheta + sensorAngles[i];

                    //Считаю от края робота
                    double sensorX = currentX + RobotRadius * Math.Sin(globalDirection);
                    double sensorY = currentY + RobotRadius * Math.Cos(globalDirection);

                    //Вычисляю координаты края объекта
                    double edgeX = sensorX + distM * Math.Sin(globalDirection);
                    double edgeY = sensorY + distM * Math.Cos(globalDirection);

                    //Проверяю край на принадлежность к стене
                    if (IsNearStaticWall(edgeX, edgeY, 0.15))
                    {
                        continue;
                    }

                    //Если это не стена, значит это динамическое препятствие
                    //Вычисляю координаты его центра для корректного хранения в памяти
                    double obsX = sensorX + (distM + ObstacleRadius) * Math.Sin(globalDirection);
                    double obsY = sensorY + (distM + ObstacleRadius) * Math.Cos(globalDirection);

                    //Ищу, есть ли уже рядом такое динамическое препятствие в памяти
                    SensedObstacle nearest = null;
                    double minDist = double.MaxValue;
                    foreach (var obs in sensedObstacles)
                    {
                        double d = Math.Sqrt(Math.Pow(obs.X - obsX, 2) + Math.Pow(obs.Y - obsY, 2));
                        if (d < minDist)
                        {
                            minDist = d;
                            nearest = obs;
                        }
                    }

                    //Если рядом (ближе 0.7 м), обновляю его время жизни
                    if (nearest != null && minDist < ObstacleMergeDistance)
                    {
                        nearest.LastSeen = now;
                    }
                    else
                    {
                        //Иначе добавляю новое динамическое препятствие в память
                        sensedObstacles.Add(new SensedObstacle { X = obsX, Y = obsY, LastSeen = now });
                    }
                }
            }
        }

        //Считаю расстояние до ближайшей стены карты в заданном направлении от робота
        private double GetDistanceToWall(double robotX, double robotY, double angle)
        {
            if (!isMapLoaded) return 999.0;

            double cellWidth = 0.25;
            double cellHeight = 0.25;
            int mapColumns = 64;
            int mapRows = 40;

            //Шагаю лучом от робота в заданном направлении
            double stepSize = 0.05; //Шаг 5 см
            double maxDistance = 5.0; //Максимум 5 метров

            for (double dist = 0; dist < maxDistance; dist += stepSize)
            {
                double checkX = robotX + dist * Math.Sin(angle);
                double checkY = robotY + dist * Math.Cos(angle);

                //Проверяю, попал ли в стену
                if (IsNearStaticWall(checkX, checkY, 0.10))
                {
                    return dist; //Вернул расстояние до стены
                }
            }

            return maxDistance; //Стены не найдено в пределах 5 метров
        }
    }
}
