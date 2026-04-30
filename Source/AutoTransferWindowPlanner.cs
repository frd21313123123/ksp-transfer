using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using UnityEngine;
using KSP.UI.Screens;

namespace AutoTransferWindowPlanner
{
    /// <summary>
    /// In-game transfer-window planner for Kerbal Space Program 1.x.
    ///
    /// The planner searches a launch date and time of flight by solving a single-revolution
    /// Lambert problem between two celestial bodies that orbit the same parent body.
    /// It then estimates departure and capture delta-v from circular parking orbits.
    ///
    /// This is intentionally dependency-free: no ToolbarController, no KAC, no external DLLs.
    /// </summary>
    [KSPAddon(KSPAddon.Startup.AllGameScenes, false)]
    public class AutoTransferWindowPlannerAddon : MonoBehaviour
    {
        private const string ModName = "Auto Transfer Window Planner";
        private const string InputLockName = "AutoTransferWindowPlanner_InputLock";

        private static AutoTransferWindowPlannerAddon instance;

        private ApplicationLauncherButton launcherButton;
        private Texture2D launcherIcon;
        private Texture2D porkchopTexture;

        private Rect windowRect = new Rect(150f, 70f, 820f, 610f);
        private Vector2 scroll;
        private bool windowVisible;

        private readonly List<CelestialBody> bodies = new List<CelestialBody>();
        private int originIndex;
        private int destinationIndex;

        private string parkingAltitudeKm = "100";
        private string captureAltitudeKm = "100";
        private string searchYears = "5";
        private string earliestLeadDays = "0";
        private string tofMinDays = "";
        private string tofMaxDays = "";
        private bool autoTof = true;
        private bool includeCapture = true;

        private readonly string[] qualityLabels = { "Быстро", "Нормально", "Точно" };
        private int qualityIndex = 1;

        private bool searchRunning;
        private bool cancelSearch;
        private float searchProgress;
        private string statusText = "Готово. Выбери планеты и нажми «Искать окно».";
        private TransferResult bestResult;
        private double[,] dvGrid;
        private int gridDepartSamples;
        private int gridTofSamples;
        private int bestGridI = -1;
        private int bestGridJ = -1;
        private double gridStartUT;
        private double gridSearchSpan;
        private double gridTofMin;
        private double gridTofMax;

        private void Awake()
        {
            if (instance != null && instance != this)
            {
                Destroy(gameObject);
                return;
            }

            instance = this;

            launcherIcon = CreateLauncherIcon();
            GameEvents.onGUIApplicationLauncherReady.Add(OnAppLauncherReady);
            GameEvents.onGUIApplicationLauncherDestroyed.Add(OnAppLauncherDestroyed);
            GameEvents.onGameSceneSwitchRequested.Add(OnGameSceneSwitchRequested);

            RefreshBodies(true);
        }

        private IEnumerator Start()
        {
            // Give KSP one frame to initialize FlightGlobals and the application launcher.
            yield return null;
            RefreshBodies(bodies.Count == 0);
            OnAppLauncherReady();
        }

        private void OnDestroy()
        {
            InputLockManager.RemoveControlLock(InputLockName);

            GameEvents.onGUIApplicationLauncherReady.Remove(OnAppLauncherReady);
            GameEvents.onGUIApplicationLauncherDestroyed.Remove(OnAppLauncherDestroyed);
            GameEvents.onGameSceneSwitchRequested.Remove(OnGameSceneSwitchRequested);

            RemoveLauncherButton();

            if (porkchopTexture != null)
            {
                Destroy(porkchopTexture);
                porkchopTexture = null;
            }

            if (launcherIcon != null)
            {
                Destroy(launcherIcon);
                launcherIcon = null;
            }

            if (instance == this)
            {
                instance = null;
            }
        }

        private void OnAppLauncherReady()
        {
            if (launcherButton != null)
            {
                return;
            }

            if (!ApplicationLauncher.Ready || ApplicationLauncher.Instance == null)
            {
                return;
            }

            launcherButton = ApplicationLauncher.Instance.AddModApplication(
                ShowWindow,
                HideWindow,
                null,
                null,
                null,
                null,
                ApplicationLauncher.AppScenes.SPACECENTER |
                ApplicationLauncher.AppScenes.FLIGHT |
                ApplicationLauncher.AppScenes.MAPVIEW |
                ApplicationLauncher.AppScenes.TRACKSTATION,
                launcherIcon);

            launcherButton.onLeftClick += ShowWindow;
            Debug.Log("[AutoTransferWindowPlanner] Toolbar button registered.");
        }

        private void OnAppLauncherDestroyed()
        {
            launcherButton = null;
        }

        private void OnGameSceneSwitchRequested(GameEvents.FromToAction<GameScenes, GameScenes> sceneSwitch)
        {
            windowVisible = false;
            InputLockManager.RemoveControlLock(InputLockName);
            RemoveLauncherButton();
        }

        private void RemoveLauncherButton()
        {
            if (launcherButton != null && ApplicationLauncher.Instance != null)
            {
                ApplicationLauncher.Instance.RemoveModApplication(launcherButton);
            }

            launcherButton = null;
        }

        private void ShowWindow()
        {
            windowVisible = true;
            windowRect = ClampWindowRect(windowRect);
            RefreshBodies(false);
            Debug.Log("[AutoTransferWindowPlanner] Window opened.");
        }

        private void HideWindow()
        {
            windowVisible = false;
            InputLockManager.RemoveControlLock(InputLockName);
        }

        private void Update()
        {
            if (!windowVisible)
            {
                return;
            }

            if (windowRect.Contains(new Vector2(Input.mousePosition.x, Screen.height - Input.mousePosition.y)))
            {
                InputLockManager.SetControlLock(ControlTypes.ALLBUTCAMERAS, InputLockName);
            }
            else
            {
                InputLockManager.RemoveControlLock(InputLockName);
            }
        }

        private void OnGUI()
        {
            if (!windowVisible)
            {
                return;
            }

            if (HighLogic.Skin != null)
            {
                GUI.skin = HighLogic.Skin;
            }

            GUI.depth = -1000;
            windowRect = ClampWindowRect(windowRect);
            windowRect = GUILayout.Window(GetInstanceID(), windowRect, DrawWindow, ModName, GUILayout.Width(820f), GUILayout.Height(610f));
        }

        private void DrawWindow(int windowId)
        {
            if (bodies.Count < 2)
            {
                RefreshBodies(false);
                GUILayout.Label("Планеты ещё не загружены. Открой сохранение/сцену KSP и попробуй снова.");
                GUI.DragWindow();
                return;
            }

            GUILayout.BeginHorizontal();
            DrawLeftPanel();
            DrawRightPanel();
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.FlexibleSpace();
            if (GUILayout.Button("Закрыть", GUILayout.Width(90f)))
            {
                HideWindow();
                if (launcherButton != null)
                {
                    launcherButton.SetFalse();
                }
            }
            GUILayout.EndHorizontal();

            GUI.DragWindow(new Rect(0f, 0f, 10000f, 28f));
        }

        private static Rect ClampWindowRect(Rect rect)
        {
            float width = Mathf.Min(820f, Mathf.Max(360f, Screen.width - 20f));
            float height = Mathf.Min(610f, Mathf.Max(260f, Screen.height - 20f));
            rect.width = width;
            rect.height = height;
            rect.x = Mathf.Clamp(rect.x, 10f, Mathf.Max(10f, Screen.width - width - 10f));
            rect.y = Mathf.Clamp(rect.y, 10f, Mathf.Max(10f, Screen.height - height - 10f));
            return rect;
        }

        private void DrawLeftPanel()
        {
            GUILayout.BeginVertical(GUILayout.Width(285f));

            GUILayout.Label("Параметры поиска", GUILayout.Height(22f));
            DrawBodySelector("Старт", ref originIndex);
            DrawBodySelector("Цель", ref destinationIndex);

            GUILayout.Space(6f);
            GUILayout.Label("Орбиты");
            DrawTextFieldRow("Начальная орбита, км", ref parkingAltitudeKm);
            DrawTextFieldRow("Финальная орбита, км", ref captureAltitudeKm);

            GUILayout.Space(6f);
            GUILayout.Label("Когда искать");
            DrawTextFieldRow("Не раньше чем через, дней", ref earliestLeadDays);
            DrawTextFieldRow("Искать ближайшие, лет", ref searchYears);

            GUILayout.Space(6f);
            GUILayout.BeginHorizontal();
            autoTof = GUILayout.Toggle(autoTof, "Авто время полёта", GUILayout.Width(170f));
            if (GUILayout.Button("?", GUILayout.Width(28f)))
            {
                statusText = "Авто TOF берёт диапазон около гомановского перелёта для выбранных орбит.";
            }
            GUILayout.EndHorizontal();

            GUI.enabled = !autoTof;
            DrawTextFieldRow("TOF min, дней", ref tofMinDays);
            DrawTextFieldRow("TOF max, дней", ref tofMaxDays);
            GUI.enabled = true;

            includeCapture = GUILayout.Toggle(includeCapture, "Считать Δv захвата у цели");

            GUILayout.BeginHorizontal();
            GUILayout.Label("Качество", GUILayout.Width(120f));
            if (GUILayout.Button(qualityLabels[qualityIndex], GUILayout.Width(110f)))
            {
                qualityIndex = (qualityIndex + 1) % qualityLabels.Length;
            }
            GUILayout.EndHorizontal();

            GUILayout.Space(10f);

            if (searchRunning)
            {
                if (GUILayout.Button("Остановить поиск", GUILayout.Height(30f)))
                {
                    cancelSearch = true;
                    statusText = "Останавливаю поиск...";
                }
            }
            else
            {
                if (GUILayout.Button("Искать окно", GUILayout.Height(34f)))
                {
                    BeginSearch();
                }
            }

            GUILayout.Space(6f);
            GUILayout.Label("Статус:");
            GUILayout.TextArea(statusText, GUILayout.Height(80f));

            GUILayout.Space(6f);
            GUILayout.Label("Поддержка:");
            GUILayout.TextArea(
                "Работает для тел с одним родительским телом: например Kerbin -> Duna, Eve -> Kerbin, Jool -> Eeloo. " +
                "Перелёты Kerbin -> Mun пока не считаются, потому что это другая задача: луна вращается вокруг Kerbin, а Kerbin вокруг Sun.",
                GUILayout.Height(85f));

            GUILayout.EndVertical();
        }

        private void DrawRightPanel()
        {
            GUILayout.BeginVertical(GUILayout.Width(505f));

            GUILayout.Label("Porkchop / карта Δv", GUILayout.Height(22f));
            DrawPorkchopArea();

            GUILayout.Space(6f);
            DrawProgressBar(searchProgress, searchRunning ? "Идёт расчёт..." : " ");

            GUILayout.Space(6f);
            GUILayout.Label("Лучший найденный вариант", GUILayout.Height(22f));
            scroll = GUILayout.BeginScrollView(scroll, GUILayout.Height(245f));
            GUILayout.TextArea(BuildResultText(), GUILayout.ExpandHeight(true));
            GUILayout.EndScrollView();

            GUILayout.EndVertical();
        }

        private void DrawBodySelector(string label, ref int index)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label(label, GUILayout.Width(92f));
            if (GUILayout.Button("<", GUILayout.Width(26f)))
            {
                index = WrapIndex(index - 1, bodies.Count);
            }
            GUILayout.Label(GetBodyName(index), GUILayout.Width(118f));
            if (GUILayout.Button(">", GUILayout.Width(26f)))
            {
                index = WrapIndex(index + 1, bodies.Count);
            }
            GUILayout.EndHorizontal();
        }

        private void DrawTextFieldRow(string label, ref string value)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label(label, GUILayout.Width(170f));
            value = GUILayout.TextField(value, GUILayout.Width(88f));
            GUILayout.EndHorizontal();
        }

        private void DrawPorkchopArea()
        {
            Rect plotRect = GUILayoutUtility.GetRect(500f, 245f, GUILayout.ExpandWidth(false));
            GUI.Box(plotRect, GUIContent.none);

            if (porkchopTexture == null)
            {
                GUI.Label(new Rect(plotRect.x + 12f, plotRect.y + 12f, plotRect.width - 24f, 60f),
                    "После расчёта здесь появится карта: X = дата старта, Y = время полёта, цвет = суммарная Δv.");
                return;
            }

            Rect texRect = new Rect(plotRect.x + 38f, plotRect.y + 10f, plotRect.width - 55f, plotRect.height - 42f);
            GUI.DrawTexture(texRect, porkchopTexture, ScaleMode.StretchToFill, false);

            GUI.Label(new Rect(plotRect.x + 8f, plotRect.y + 6f, 50f, 20f), "TOF");
            GUI.Label(new Rect(plotRect.x + 190f, plotRect.y + plotRect.height - 24f, 220f, 20f), "Дата старта");

            if (bestResult != null)
            {
                GUI.Label(new Rect(plotRect.x + 8f, plotRect.y + plotRect.height - 22f, 180f, 20f),
                    "min: " + FormatDV(bestResult.TotalDV));
            }
        }

        private void DrawProgressBar(float value, string text)
        {
            Rect rect = GUILayoutUtility.GetRect(500f, 20f);
            GUI.Box(rect, GUIContent.none);
            float clamped = Mathf.Clamp01(value);
            Rect filled = new Rect(rect.x + 2f, rect.y + 2f, (rect.width - 4f) * clamped, rect.height - 4f);
            GUI.Box(filled, GUIContent.none);
            GUI.Label(rect, text + " " + Math.Round(clamped * 100f).ToString(CultureInfo.InvariantCulture) + "%");
        }

        private string BuildResultText()
        {
            if (bestResult == null)
            {
                return "Пока результата нет. Нажми «Искать окно».";
            }

            string longWayText = bestResult.LongWay ? "да" : "нет";
            string captureLine = includeCapture
                ? "Δv захвата:       " + FormatDV(bestResult.InsertionDV) + "\n"
                : "Δv захвата:       не включено в оптимизацию\n";

            return
                bestResult.Origin.bodyName + " -> " + bestResult.Destination.bodyName + "\n" +
                "\n" +
                "Старт:            " + FormatUT(bestResult.DepartureUT) + "\n" +
                "Прибытие:         " + FormatUT(bestResult.ArrivalUT) + "\n" +
                "Время полёта:     " + FormatDuration(bestResult.TimeOfFlight) + "\n" +
                "\n" +
                "Итого Δv:         " + FormatDV(bestResult.TotalDV) + "\n" +
                "Δv вылета:        " + FormatDV(bestResult.EjectionDV) + "\n" +
                captureLine +
                "v∞ старта:        " + FormatSpeed(bestResult.DepartureVInf) + "\n" +
                "v∞ прибытия:      " + FormatSpeed(bestResult.ArrivalVInf) + "\n" +
                "C3 старта:        " + FormatC3(bestResult.DepartureVInf) + "\n" +
                "\n" +
                "Фазовый угол:     " + FormatAngle(bestResult.PhaseAngleDeg) + "\n" +
                "Угол вылета:      " + FormatAngle(bestResult.EjectionAngleDeg) + "\n" +
                "Наклон вылета:    " + FormatAngle(bestResult.EjectionInclinationDeg) + "\n" +
                "Long way:         " + longWayText + "\n" +
                "\n" +
                "Примечание: это оценка patched-conics. Атмосфера, плоскость стартовой площадки, наклон парковочной орбиты, аэроторможение и mid-course corrections здесь не моделируются.";
        }

        private void BeginSearch()
        {
            RefreshBodies(false);

            if (bodies.Count < 2)
            {
                statusText = "Не удалось найти тела в текущей игре.";
                return;
            }

            CelestialBody origin = bodies[WrapIndex(originIndex, bodies.Count)];
            CelestialBody destination = bodies[WrapIndex(destinationIndex, bodies.Count)];

            if (origin == destination)
            {
                statusText = "Старт и цель должны быть разными.";
                return;
            }

            if (origin.orbit == null || destination.orbit == null || origin.orbit.referenceBody == null || destination.orbit.referenceBody == null)
            {
                statusText = "Одно из выбранных тел не имеет орбиты. Обычно это звезда/центр системы.";
                return;
            }

            if (origin.orbit.referenceBody != destination.orbit.referenceBody)
            {
                statusText = "Пока поддерживаются только тела с одним родительским телом. Например Kerbin -> Duna, но не Kerbin -> Mun.";
                return;
            }

            double parkingAlt = ParseDouble(parkingAltitudeKm, 100.0) * 1000.0;
            double captureAlt = ParseDouble(captureAltitudeKm, 100.0) * 1000.0;
            double lead = Math.Max(0.0, ParseDouble(earliestLeadDays, 0.0)) * DaySeconds();
            double years = Math.Max(0.02, ParseDouble(searchYears, 5.0));
            double searchSpan = years * YearSeconds();
            double startUT = Planetarium.GetUniversalTime() + lead;

            double minTof;
            double maxTof;
            if (autoTof || string.IsNullOrEmpty(tofMinDays.Trim()) || string.IsNullOrEmpty(tofMaxDays.Trim()))
            {
                GetAutomaticTofRange(origin, destination, out minTof, out maxTof);
                tofMinDays = FormatNumber(minTof / DaySeconds(), 1);
                tofMaxDays = FormatNumber(maxTof / DaySeconds(), 1);
            }
            else
            {
                minTof = Math.Max(0.05, ParseDouble(tofMinDays, 1.0)) * DaySeconds();
                maxTof = Math.Max(minTof + DaySeconds() * 0.05, ParseDouble(tofMaxDays, minTof / DaySeconds() + 10.0) * DaySeconds());
            }

            int depSamples;
            int tofSamples;
            GetQualitySamples(out depSamples, out tofSamples);

            bestResult = null;
            dvGrid = null;
            bestGridI = -1;
            bestGridJ = -1;
            searchProgress = 0f;
            cancelSearch = false;
            searchRunning = true;
            statusText = "Начинаю поиск...";

            if (porkchopTexture != null)
            {
                Destroy(porkchopTexture);
                porkchopTexture = null;
            }

            SearchRequest request = new SearchRequest
            {
                Origin = origin,
                Destination = destination,
                ParkingAltitude = Math.Max(0.0, parkingAlt),
                CaptureAltitude = Math.Max(0.0, captureAlt),
                StartUT = startUT,
                SearchSpan = searchSpan,
                TofMin = minTof,
                TofMax = maxTof,
                DepartSamples = depSamples,
                TofSamples = tofSamples,
                IncludeCapture = includeCapture
            };

            StartCoroutine(SearchCoroutine(request));
        }

        private IEnumerator SearchCoroutine(SearchRequest request)
        {
            gridDepartSamples = request.DepartSamples;
            gridTofSamples = request.TofSamples;
            gridStartUT = request.StartUT;
            gridSearchSpan = request.SearchSpan;
            gridTofMin = request.TofMin;
            gridTofMax = request.TofMax;
            dvGrid = new double[gridDepartSamples, gridTofSamples];

            for (int i = 0; i < gridDepartSamples; i++)
            {
                for (int j = 0; j < gridTofSamples; j++)
                {
                    dvGrid[i, j] = double.NaN;
                }
            }

            TransferResult best = null;
            double bestScore = double.PositiveInfinity;
            int evalCount = 0;
            int totalCount = Math.Max(1, gridDepartSamples * gridTofSamples);

            statusText = "Грубый поиск: " + request.Origin.bodyName + " -> " + request.Destination.bodyName;

            for (int i = 0; i < gridDepartSamples; i++)
            {
                if (cancelSearch)
                {
                    break;
                }

                double departUT = request.StartUT + request.SearchSpan * i / Math.Max(1, gridDepartSamples - 1);

                for (int j = 0; j < gridTofSamples; j++)
                {
                    if (cancelSearch)
                    {
                        break;
                    }

                    double tof = request.TofMin + (request.TofMax - request.TofMin) * j / Math.Max(1, gridTofSamples - 1);
                    TransferResult result;
                    if (TryEvaluateTransfer(request, departUT, tof, out result))
                    {
                        dvGrid[i, j] = result.TotalDV;
                        if (result.TotalDV < bestScore)
                        {
                            bestScore = result.TotalDV;
                            best = result;
                            bestGridI = i;
                            bestGridJ = j;
                        }
                    }

                    evalCount++;
                    if ((evalCount & 31) == 0)
                    {
                        searchProgress = 0.82f * evalCount / totalCount;
                        yield return null;
                    }
                }
            }

            if (!cancelSearch && best != null)
            {
                best = RefineBest(request, best, 3);
                bestResult = best;
                statusText = "Готово. Лучшее окно найдено: " + FormatDV(best.TotalDV) + ".";
            }
            else if (cancelSearch)
            {
                statusText = "Поиск остановлен.";
            }
            else
            {
                statusText = "Не удалось найти решение. Попробуй увеличить годы поиска или диапазон TOF.";
            }

            BuildPorkchopTexture();
            searchProgress = 1f;
            searchRunning = false;
        }

        private TransferResult RefineBest(SearchRequest request, TransferResult seed, int passes)
        {
            TransferResult best = seed;
            double depStep = request.SearchSpan / Math.Max(1, request.DepartSamples - 1);
            double tofStep = (request.TofMax - request.TofMin) / Math.Max(1, request.TofSamples - 1);

            for (int pass = 0; pass < passes; pass++)
            {
                double bestScore = best.TotalDV;
                TransferResult passBest = best;
                int samples = 13;
                double depRange = depStep * 1.6;
                double tofRange = tofStep * 1.6;

                for (int i = 0; i < samples; i++)
                {
                    double depOffset = -depRange + 2.0 * depRange * i / (samples - 1);
                    double depUT = best.DepartureUT + depOffset;
                    if (depUT < request.StartUT || depUT > request.StartUT + request.SearchSpan)
                    {
                        continue;
                    }

                    for (int j = 0; j < samples; j++)
                    {
                        double tofOffset = -tofRange + 2.0 * tofRange * j / (samples - 1);
                        double tof = best.TimeOfFlight + tofOffset;
                        if (tof < request.TofMin || tof > request.TofMax)
                        {
                            continue;
                        }

                        TransferResult result;
                        if (TryEvaluateTransfer(request, depUT, tof, out result) && result.TotalDV < bestScore)
                        {
                            bestScore = result.TotalDV;
                            passBest = result;
                        }
                    }
                }

                best = passBest;
                depStep *= 0.28;
                tofStep *= 0.28;
            }

            return best;
        }

        private bool TryEvaluateTransfer(SearchRequest request, double departUT, double tof, out TransferResult bestResultForPoint)
        {
            bestResultForPoint = null;

            if (tof <= 0.0)
            {
                return false;
            }

            double arrivalUT = departUT + tof;
            CelestialBody origin = request.Origin;
            CelestialBody destination = request.Destination;
            CelestialBody central = origin.orbit.referenceBody;

            Vector3d r1 = origin.orbit.getRelativePositionAtUT(departUT);
            Vector3d r2 = destination.orbit.getRelativePositionAtUT(arrivalUT);
            Vector3d originVel = origin.orbit.getOrbitalVelocityAtUT(departUT);
            Vector3d destVel = destination.orbit.getOrbitalVelocityAtUT(arrivalUT);

            TransferResult best = null;
            double bestScore = double.PositiveInfinity;

            for (int longWayValue = 0; longWayValue < 2; longWayValue++)
            {
                bool longWay = longWayValue == 1;
                Vector3d transferV1;
                Vector3d transferV2;

                if (!LambertSolver.TrySolve(r1, r2, tof, central.gravParameter, longWay, out transferV1, out transferV2))
                {
                    continue;
                }

                Vector3d vinfDepartureVector = transferV1 - originVel;
                Vector3d vinfArrivalVector = transferV2 - destVel;
                double vinfDeparture = vinfDepartureVector.magnitude;
                double vinfArrival = vinfArrivalVector.magnitude;

                if (!IsFinite(vinfDeparture) || !IsFinite(vinfArrival))
                {
                    continue;
                }

                double ejection = CircularOrbitToHyperbolaDV(origin, request.ParkingAltitude, vinfDeparture);
                double insertion = CircularOrbitToHyperbolaDV(destination, request.CaptureAltitude, vinfArrival);
                double total = ejection + (request.IncludeCapture ? insertion : 0.0);

                if (!IsFinite(total) || total <= 0.0)
                {
                    continue;
                }

                TransferResult candidate = new TransferResult
                {
                    Origin = origin,
                    Destination = destination,
                    DepartureUT = departUT,
                    ArrivalUT = arrivalUT,
                    TimeOfFlight = tof,
                    TotalDV = total,
                    EjectionDV = ejection,
                    InsertionDV = insertion,
                    DepartureVInf = vinfDeparture,
                    ArrivalVInf = vinfArrival,
                    LongWay = longWay,
                    PhaseAngleDeg = SignedAngleDeg(r1, destination.orbit.getRelativePositionAtUT(departUT), SafeNormal(r1, originVel)),
                    EjectionAngleDeg = SignedAngleDeg(originVel, vinfDepartureVector, SafeNormal(r1, originVel)),
                    EjectionInclinationDeg = InclinationToPlaneDeg(vinfDepartureVector, SafeNormal(r1, originVel))
                };

                if (candidate.TotalDV < bestScore)
                {
                    bestScore = candidate.TotalDV;
                    best = candidate;
                }
            }

            if (best == null)
            {
                return false;
            }

            bestResultForPoint = best;
            return true;
        }

        private void BuildPorkchopTexture()
        {
            if (dvGrid == null || gridDepartSamples <= 0 || gridTofSamples <= 0)
            {
                return;
            }

            if (porkchopTexture != null)
            {
                Destroy(porkchopTexture);
                porkchopTexture = null;
            }

            double min = double.PositiveInfinity;
            double max = double.NegativeInfinity;

            for (int i = 0; i < gridDepartSamples; i++)
            {
                for (int j = 0; j < gridTofSamples; j++)
                {
                    double v = dvGrid[i, j];
                    if (IsFinite(v))
                    {
                        if (v < min) min = v;
                        if (v > max) max = v;
                    }
                }
            }

            if (!IsFinite(min) || !IsFinite(max) || max <= min)
            {
                return;
            }

            porkchopTexture = new Texture2D(gridDepartSamples, gridTofSamples, TextureFormat.RGB24, false);
            porkchopTexture.filterMode = FilterMode.Point;

            double logMin = Math.Log(Math.Max(1.0, min));
            double logMax = Math.Log(Math.Max(2.0, max));
            double logSpan = Math.Max(1e-6, logMax - logMin);

            for (int i = 0; i < gridDepartSamples; i++)
            {
                for (int j = 0; j < gridTofSamples; j++)
                {
                    double v = dvGrid[i, j];
                    Color color;
                    if (!IsFinite(v))
                    {
                        color = new Color(0.05f, 0.05f, 0.05f, 1f);
                    }
                    else
                    {
                        double t = (Math.Log(Math.Max(1.0, v)) - logMin) / logSpan;
                        color = HeatColor((float)Clamp01(t));
                    }

                    porkchopTexture.SetPixel(i, j, color);
                }
            }

            if (bestGridI >= 0 && bestGridJ >= 0)
            {
                for (int dx = -2; dx <= 2; dx++)
                {
                    SetGridPixelSafe(bestGridI + dx, bestGridJ, Color.white);
                    SetGridPixelSafe(bestGridI, bestGridJ + dx, Color.white);
                }
            }

            porkchopTexture.Apply(false, false);
        }

        private void SetGridPixelSafe(int x, int y, Color color)
        {
            if (porkchopTexture == null)
            {
                return;
            }

            if (x < 0 || y < 0 || x >= porkchopTexture.width || y >= porkchopTexture.height)
            {
                return;
            }

            porkchopTexture.SetPixel(x, y, color);
        }

        private void RefreshBodies(bool resetSelection)
        {
            if (FlightGlobals.Bodies == null)
            {
                return;
            }

            CelestialBody previousOrigin = null;
            CelestialBody previousDestination = null;
            if (bodies.Count > 0)
            {
                previousOrigin = bodies[WrapIndex(originIndex, bodies.Count)];
                previousDestination = bodies[WrapIndex(destinationIndex, bodies.Count)];
            }

            bodies.Clear();
            foreach (CelestialBody body in FlightGlobals.Bodies)
            {
                if (body == null || body.orbit == null || body.orbit.referenceBody == null)
                {
                    continue;
                }

                bodies.Add(body);
            }

            bodies.Sort(delegate (CelestialBody a, CelestialBody b)
            {
                double aa = a.orbit != null ? Math.Abs(a.orbit.semiMajorAxis) : 0.0;
                double bb = b.orbit != null ? Math.Abs(b.orbit.semiMajorAxis) : 0.0;
                return aa.CompareTo(bb);
            });

            if (bodies.Count == 0)
            {
                originIndex = 0;
                destinationIndex = 0;
                return;
            }

            if (resetSelection)
            {
                originIndex = FindBodyIndexByName("Kerbin");
                if (originIndex < 0 && FlightGlobals.ActiveVessel != null && FlightGlobals.ActiveVessel.mainBody != null)
                {
                    originIndex = bodies.IndexOf(FlightGlobals.ActiveVessel.mainBody);
                }
                if (originIndex < 0) originIndex = 0;

                destinationIndex = FindBodyIndexByName("Duna");
                if (destinationIndex < 0 || destinationIndex == originIndex) destinationIndex = WrapIndex(originIndex + 1, bodies.Count);
            }
            else
            {
                if (previousOrigin != null && bodies.Contains(previousOrigin)) originIndex = bodies.IndexOf(previousOrigin);
                else originIndex = WrapIndex(originIndex, bodies.Count);

                if (previousDestination != null && bodies.Contains(previousDestination)) destinationIndex = bodies.IndexOf(previousDestination);
                else destinationIndex = WrapIndex(destinationIndex, bodies.Count);

                if (destinationIndex == originIndex)
                {
                    destinationIndex = WrapIndex(originIndex + 1, bodies.Count);
                }
            }
        }

        private int FindBodyIndexByName(string bodyName)
        {
            for (int i = 0; i < bodies.Count; i++)
            {
                if (string.Equals(bodies[i].bodyName, bodyName, StringComparison.OrdinalIgnoreCase) ||
                    string.Equals(bodies[i].name, bodyName, StringComparison.OrdinalIgnoreCase))
                {
                    return i;
                }
            }

            return -1;
        }

        private string GetBodyName(int index)
        {
            if (bodies.Count == 0)
            {
                return "—";
            }

            return bodies[WrapIndex(index, bodies.Count)].bodyName;
        }

        private static int WrapIndex(int index, int count)
        {
            if (count <= 0)
            {
                return 0;
            }

            while (index < 0) index += count;
            while (index >= count) index -= count;
            return index;
        }

        private void GetQualitySamples(out int departSamples, out int tofSamples)
        {
            if (qualityIndex <= 0)
            {
                departSamples = 48;
                tofSamples = 40;
            }
            else if (qualityIndex == 1)
            {
                departSamples = 76;
                tofSamples = 58;
            }
            else
            {
                departSamples = 120;
                tofSamples = 88;
            }
        }

        private static void GetAutomaticTofRange(CelestialBody origin, CelestialBody destination, out double minTof, out double maxTof)
        {
            double day = DaySeconds();
            double mu = origin.orbit.referenceBody.gravParameter;
            double r1 = Math.Abs(origin.orbit.semiMajorAxis);
            double r2 = Math.Abs(destination.orbit.semiMajorAxis);
            double a = 0.5 * (r1 + r2);
            double hohmann = Math.PI * Math.Sqrt(a * a * a / mu);

            minTof = Math.Max(day * 5.0, hohmann * 0.38);
            maxTof = Math.Max(minTof + day * 10.0, hohmann * 1.85);
        }

        private static double CircularOrbitToHyperbolaDV(CelestialBody body, double altitude, double vinf)
        {
            double radius = Math.Max(body.Radius + altitude, body.Radius + 1.0);
            double mu = body.gravParameter;
            double circular = Math.Sqrt(mu / radius);
            double hyperbola = Math.Sqrt(vinf * vinf + 2.0 * mu / radius);
            return Math.Max(0.0, hyperbola - circular);
        }

        private static Vector3d SafeNormal(Vector3d position, Vector3d velocity)
        {
            Vector3d h = Vector3d.Cross(position, velocity);
            if (h.sqrMagnitude < 1e-16)
            {
                return new Vector3d(0.0, 0.0, 1.0);
            }

            return h.normalized;
        }

        private static double SignedAngleDeg(Vector3d from, Vector3d to, Vector3d normal)
        {
            if (from.sqrMagnitude < 1e-16 || to.sqrMagnitude < 1e-16)
            {
                return 0.0;
            }

            Vector3d f = from.normalized;
            Vector3d t = to.normalized;
            Vector3d n = normal.normalized;
            double x = Clamp(Vector3d.Dot(f, t), -1.0, 1.0);
            double y = Vector3d.Dot(n, Vector3d.Cross(f, t));
            double deg = Math.Atan2(y, x) * 180.0 / Math.PI;
            if (deg < 0.0) deg += 360.0;
            return deg;
        }

        private static double InclinationToPlaneDeg(Vector3d vector, Vector3d planeNormal)
        {
            if (vector.sqrMagnitude < 1e-16)
            {
                return 0.0;
            }

            double s = Clamp(Vector3d.Dot(vector.normalized, planeNormal.normalized), -1.0, 1.0);
            return Math.Asin(s) * 180.0 / Math.PI;
        }

        private static double ParseDouble(string text, double defaultValue)
        {
            if (string.IsNullOrEmpty(text))
            {
                return defaultValue;
            }

            double value;
            string normalized = text.Trim().Replace(',', '.');
            if (double.TryParse(normalized, NumberStyles.Float, CultureInfo.InvariantCulture, out value))
            {
                return value;
            }

            return defaultValue;
        }

        private static string FormatUT(double ut)
        {
            double time = Math.Max(0.0, ut);
            int year = (int)Math.Floor(time / YearSeconds()) + 1;
            time %= YearSeconds();
            int day = (int)Math.Floor(time / DaySeconds()) + 1;
            time %= DaySeconds();
            int hour = (int)Math.Floor(time / HourSeconds());
            time %= HourSeconds();
            int minute = (int)Math.Floor(time / MinuteSeconds());
            time %= MinuteSeconds();
            int second = (int)Math.Floor(time);

            return "Y" + year + ", D" + day + ", " + hour.ToString("00") + ":" + minute.ToString("00") + ":" + second.ToString("00");
        }

        private static string FormatDuration(double seconds)
        {
            double time = Math.Max(0.0, seconds);
            int days = (int)Math.Floor(time / DaySeconds());
            time %= DaySeconds();
            int hour = (int)Math.Floor(time / HourSeconds());
            time %= HourSeconds();
            int minute = (int)Math.Floor(time / MinuteSeconds());
            return days + " д " + hour.ToString("00") + ":" + minute.ToString("00");
        }

        private static string FormatDV(double value)
        {
            return FormatNumber(value, 0) + " м/с";
        }

        private static string FormatSpeed(double value)
        {
            return FormatNumber(value, 1) + " м/с";
        }

        private static string FormatC3(double vinfMetersPerSecond)
        {
            double kmps = vinfMetersPerSecond / 1000.0;
            return FormatNumber(kmps * kmps, 3) + " км²/с²";
        }

        private static string FormatAngle(double deg)
        {
            return FormatNumber(deg, 2) + "°";
        }

        private static string FormatNumber(double value, int decimals)
        {
            if (!IsFinite(value))
            {
                return "—";
            }

            return value.ToString("F" + decimals, CultureInfo.InvariantCulture);
        }

        private static double DaySeconds()
        {
            return KSPUtil.dateTimeFormatter.Day > 0.0 ? KSPUtil.dateTimeFormatter.Day : 21600.0;
        }

        private static double YearSeconds()
        {
            return KSPUtil.dateTimeFormatter.Year > 0.0 ? KSPUtil.dateTimeFormatter.Year : DaySeconds() * 426.0;
        }

        private static double HourSeconds()
        {
            return KSPUtil.dateTimeFormatter.Hour > 0.0 ? KSPUtil.dateTimeFormatter.Hour : 3600.0;
        }

        private static double MinuteSeconds()
        {
            return KSPUtil.dateTimeFormatter.Minute > 0.0 ? KSPUtil.dateTimeFormatter.Minute : 60.0;
        }

        private static bool IsFinite(double value)
        {
            return !double.IsNaN(value) && !double.IsInfinity(value);
        }

        private static double Clamp(double value, double min, double max)
        {
            if (value < min) return min;
            if (value > max) return max;
            return value;
        }

        private static double Clamp01(double value)
        {
            return Clamp(value, 0.0, 1.0);
        }

        private static Color HeatColor(float t)
        {
            t = Mathf.Clamp01(t);
            if (t < 0.25f)
            {
                return Color.Lerp(new Color(0.05f, 0.10f, 0.95f), new Color(0.00f, 0.85f, 1.00f), t / 0.25f);
            }
            if (t < 0.50f)
            {
                return Color.Lerp(new Color(0.00f, 0.85f, 1.00f), new Color(0.10f, 0.95f, 0.20f), (t - 0.25f) / 0.25f);
            }
            if (t < 0.75f)
            {
                return Color.Lerp(new Color(0.10f, 0.95f, 0.20f), new Color(1.00f, 0.95f, 0.05f), (t - 0.50f) / 0.25f);
            }

            return Color.Lerp(new Color(1.00f, 0.95f, 0.05f), new Color(1.00f, 0.18f, 0.05f), (t - 0.75f) / 0.25f);
        }

        private static Texture2D CreateLauncherIcon()
        {
            Texture2D texture = new Texture2D(38, 38, TextureFormat.RGBA32, false);
            texture.filterMode = FilterMode.Point;

            Color background = new Color(0.05f, 0.07f, 0.10f, 1f);
            Color orbit = new Color(0.35f, 0.80f, 1.00f, 1f);
            Color transfer = new Color(1.00f, 0.85f, 0.10f, 1f);
            Color body = new Color(0.80f, 0.90f, 1.00f, 1f);

            for (int y = 0; y < 38; y++)
            {
                for (int x = 0; x < 38; x++)
                {
                    texture.SetPixel(x, y, background);
                }
            }

            for (int a = 0; a < 360; a++)
            {
                double rad = a * Math.PI / 180.0;
                int x1 = 19 + (int)Math.Round(Math.Cos(rad) * 14.0);
                int y1 = 19 + (int)Math.Round(Math.Sin(rad) * 14.0);
                int x2 = 19 + (int)Math.Round(Math.Cos(rad) * 8.0);
                int y2 = 19 + (int)Math.Round(Math.Sin(rad) * 8.0);
                if (x1 >= 0 && y1 >= 0 && x1 < 38 && y1 < 38) texture.SetPixel(x1, y1, orbit);
                if (x2 >= 0 && y2 >= 0 && x2 < 38 && y2 < 38) texture.SetPixel(x2, y2, orbit);
            }

            for (int x = 8; x <= 30; x++)
            {
                double u = (x - 8) / 22.0;
                int y = 10 + (int)Math.Round(18.0 * u + 5.0 * Math.Sin(u * Math.PI));
                if (x >= 0 && y >= 0 && x < 38 && y < 38) texture.SetPixel(x, y, transfer);
                if (x >= 0 && y + 1 >= 0 && x < 38 && y + 1 < 38) texture.SetPixel(x, y + 1, transfer);
            }

            DrawFilledCircle(texture, 11, 12, 3, body);
            DrawFilledCircle(texture, 29, 29, 4, body);
            texture.Apply(false, false);
            return texture;
        }

        private static void DrawFilledCircle(Texture2D texture, int cx, int cy, int radius, Color color)
        {
            for (int y = -radius; y <= radius; y++)
            {
                for (int x = -radius; x <= radius; x++)
                {
                    if (x * x + y * y > radius * radius)
                    {
                        continue;
                    }

                    int px = cx + x;
                    int py = cy + y;
                    if (px >= 0 && py >= 0 && px < texture.width && py < texture.height)
                    {
                        texture.SetPixel(px, py, color);
                    }
                }
            }
        }

        private sealed class SearchRequest
        {
            public CelestialBody Origin;
            public CelestialBody Destination;
            public double ParkingAltitude;
            public double CaptureAltitude;
            public double StartUT;
            public double SearchSpan;
            public double TofMin;
            public double TofMax;
            public int DepartSamples;
            public int TofSamples;
            public bool IncludeCapture;
        }

        private sealed class TransferResult
        {
            public CelestialBody Origin;
            public CelestialBody Destination;
            public double DepartureUT;
            public double ArrivalUT;
            public double TimeOfFlight;
            public double TotalDV;
            public double EjectionDV;
            public double InsertionDV;
            public double DepartureVInf;
            public double ArrivalVInf;
            public double PhaseAngleDeg;
            public double EjectionAngleDeg;
            public double EjectionInclinationDeg;
            public bool LongWay;
        }
    }

    internal static class LambertSolver
    {
        public static bool TrySolve(Vector3d r1, Vector3d r2, double timeOfFlight, double mu, bool longWay, out Vector3d v1, out Vector3d v2)
        {
            v1 = Vector3d.zero;
            v2 = Vector3d.zero;

            if (timeOfFlight <= 0.0 || mu <= 0.0)
            {
                return false;
            }

            double r1m = r1.magnitude;
            double r2m = r2.magnitude;
            if (r1m <= 0.0 || r2m <= 0.0)
            {
                return false;
            }

            double cosDtheta = Clamp(Vector3d.Dot(r1, r2) / (r1m * r2m), -1.0, 1.0);
            double dtheta = Math.Acos(cosDtheta);
            if (longWay)
            {
                dtheta = 2.0 * Math.PI - dtheta;
            }

            double sinDtheta = Math.Sin(dtheta);
            if (Math.Abs(sinDtheta) < 1e-10)
            {
                return false;
            }

            double denominator = 1.0 - Math.Cos(dtheta);
            if (denominator <= 0.0)
            {
                return false;
            }

            double A = sinDtheta * Math.Sqrt(r1m * r2m / denominator);
            if (Math.Abs(A) < 1e-12 || !IsFinite(A))
            {
                return false;
            }

            double z0;
            double z1;
            if (!FindBracket(r1m, r2m, A, mu, timeOfFlight, out z0, out z1))
            {
                return false;
            }

            double f0;
            if (!TryF(z0, r1m, r2m, A, mu, timeOfFlight, out f0))
            {
                return false;
            }

            double z = 0.5 * (z0 + z1);
            for (int iter = 0; iter < 110; iter++)
            {
                z = 0.5 * (z0 + z1);
                double f;
                if (!TryF(z, r1m, r2m, A, mu, timeOfFlight, out f))
                {
                    z0 = z;
                    continue;
                }

                if (Math.Abs(f) < 1e-5 || Math.Abs(z1 - z0) < 1e-9)
                {
                    break;
                }

                if (f0 * f <= 0.0)
                {
                    z1 = z;
                }
                else
                {
                    z0 = z;
                    f0 = f;
                }
            }

            double y;
            if (!TryY(z, r1m, r2m, A, out y))
            {
                return false;
            }

            double fLambert = 1.0 - y / r1m;
            double g = A * Math.Sqrt(y / mu);
            double gdot = 1.0 - y / r2m;
            if (Math.Abs(g) < 1e-12 || !IsFinite(g))
            {
                return false;
            }

            v1 = (r2 - r1 * fLambert) / g;
            v2 = (r2 * gdot - r1) / g;

            return IsFinite(v1.x) && IsFinite(v1.y) && IsFinite(v1.z) &&
                   IsFinite(v2.x) && IsFinite(v2.y) && IsFinite(v2.z);
        }

        private static bool FindBracket(double r1m, double r2m, double A, double mu, double targetTime, out double z0, out double z1)
        {
            z0 = 0.0;
            z1 = 0.0;

            // Fast path: the single-revolution Lambert time is usually bracketed in [-4π², 4π²].
            double low = -4.0 * Math.PI * Math.PI + 1e-6;
            double high = 4.0 * Math.PI * Math.PI;
            double fLow = 0.0;
            bool lowOk = false;
            for (int i = 0; i < 80; i++)
            {
                if (TryF(low, r1m, r2m, A, mu, targetTime, out fLow))
                {
                    lowOk = true;
                    break;
                }
                low += 0.5;
            }

            if (lowOk)
            {
                double fHigh;
                double highTry = high;
                for (int i = 0; i < 6; i++)
                {
                    if (TryF(highTry, r1m, r2m, A, mu, targetTime, out fHigh) && fLow * fHigh <= 0.0)
                    {
                        z0 = low;
                        z1 = highTry;
                        return true;
                    }

                    highTry *= 2.0;
                }
            }

            // Robust fallback: scan several ranges and use the first sign change.
            double previousZ = 0.0;
            double previousF = 0.0;
            bool hasPrevious = false;

            double[] mins = { -4.0 * Math.PI * Math.PI, 4.0 * Math.PI * Math.PI, 16.0 * Math.PI * Math.PI };
            double[] maxs = {  4.0 * Math.PI * Math.PI, 16.0 * Math.PI * Math.PI, 64.0 * Math.PI * Math.PI };
            int[] counts = { 90, 90, 130 };

            for (int range = 0; range < mins.Length; range++)
            {
                double min = mins[range];
                double max = maxs[range];
                int count = counts[range];

                for (int k = 0; k < count; k++)
                {
                    double z = min + (max - min) * k / Math.Max(1, count - 1);
                    double f;
                    if (!TryF(z, r1m, r2m, A, mu, targetTime, out f))
                    {
                        hasPrevious = false;
                        continue;
                    }

                    if (Math.Abs(f) < 1e-5)
                    {
                        z0 = z - 1e-5;
                        z1 = z + 1e-5;
                        return true;
                    }

                    if (hasPrevious && previousF * f <= 0.0)
                    {
                        z0 = previousZ;
                        z1 = z;
                        return true;
                    }

                    previousZ = z;
                    previousF = f;
                    hasPrevious = true;
                }
            }

            return false;
        }

        private static bool TryF(double z, double r1m, double r2m, double A, double mu, double targetTime, out double f)
        {
            f = 0.0;
            double tof;
            if (!TryTimeOfFlight(z, r1m, r2m, A, mu, out tof))
            {
                return false;
            }

            f = tof - targetTime;
            return IsFinite(f);
        }

        private static bool TryTimeOfFlight(double z, double r1m, double r2m, double A, double mu, out double tof)
        {
            tof = 0.0;
            double y;
            if (!TryY(z, r1m, r2m, A, out y))
            {
                return false;
            }

            double C = StumpffC(z);
            double S = StumpffS(z);
            if (C <= 0.0 || !IsFinite(C) || !IsFinite(S))
            {
                return false;
            }

            double x = Math.Sqrt(y / C);
            tof = (x * x * x * S + A * Math.Sqrt(y)) / Math.Sqrt(mu);
            return IsFinite(tof) && tof >= 0.0;
        }

        private static bool TryY(double z, double r1m, double r2m, double A, out double y)
        {
            y = 0.0;
            double C = StumpffC(z);
            double S = StumpffS(z);
            if (C <= 0.0 || !IsFinite(C) || !IsFinite(S))
            {
                return false;
            }

            y = r1m + r2m + A * (z * S - 1.0) / Math.Sqrt(C);
            return IsFinite(y) && y >= 0.0;
        }

        private static double StumpffC(double z)
        {
            if (z > 1e-8)
            {
                double s = Math.Sqrt(z);
                return (1.0 - Math.Cos(s)) / z;
            }

            if (z < -1e-8)
            {
                double s = Math.Sqrt(-z);
                if (s > 50.0)
                {
                    return double.PositiveInfinity;
                }
                return (Math.Cosh(s) - 1.0) / (-z);
            }

            return 0.5 - z / 24.0 + z * z / 720.0;
        }

        private static double StumpffS(double z)
        {
            if (z > 1e-8)
            {
                double s = Math.Sqrt(z);
                return (s - Math.Sin(s)) / (s * s * s);
            }

            if (z < -1e-8)
            {
                double s = Math.Sqrt(-z);
                if (s > 50.0)
                {
                    return double.PositiveInfinity;
                }
                return (Math.Sinh(s) - s) / (s * s * s);
            }

            return 1.0 / 6.0 - z / 120.0 + z * z / 5040.0;
        }

        private static double Clamp(double value, double min, double max)
        {
            if (value < min) return min;
            if (value > max) return max;
            return value;
        }

        private static bool IsFinite(double value)
        {
            return !double.IsNaN(value) && !double.IsInfinity(value);
        }
    }
}
