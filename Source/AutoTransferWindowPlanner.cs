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
    /// Lambert problem in the best available patched-conics frame for the selected bodies.
    /// It then estimates departure and capture delta-v from circular parking orbits.
    ///
    /// This is intentionally dependency-free: no ToolbarController, no KAC, no external DLLs.
    /// </summary>
    [KSPAddon(KSPAddon.Startup.AllGameScenes, false)]
    public class AutoTransferWindowPlannerAddon : MonoBehaviour
    {
        private const string ModName = "Auto Transfer Window Planner";
        private const string InputLockName = "AutoTransferWindowPlanner_InputLock";
        private const double SearchFrameBudgetSeconds = 0.006;
        private const string SingleRevWarning = " Solver ищет только single-rev Lambert; резонансные/multi-rev окна могут быть пропущены.";

        private static readonly double[] LegDurationFactorsFast = { 0.45, 0.65, 0.85, 1.08, 1.38, 1.80, 2.35 };
        private static readonly double[] LegDurationFactorsNormal = { 0.36, 0.48, 0.62, 0.78, 0.95, 1.15, 1.38, 1.68, 2.05, 2.55, 3.15 };
        private static readonly double[] LegDurationFactorsAccurate = { 0.30, 0.38, 0.48, 0.60, 0.74, 0.90, 1.08, 1.30, 1.55, 1.85, 2.20, 2.65, 3.20, 3.90 };
        private static readonly double[] LegDurationSeedFactors = { 0.88, 0.94, 0.98, 1.0, 1.02, 1.06, 1.14 };

        private static AutoTransferWindowPlannerAddon instance;

        private ApplicationLauncherButton launcherButton;
        private Texture2D launcherIcon;
        private Texture2D porkchopTexture;

        private Rect windowRect = new Rect(150f, 70f, 820f, 610f);
        private Vector2 leftScroll;
        private Vector2 scroll;
        private bool windowVisible;

        private readonly List<CelestialBody> bodies = new List<CelestialBody>();
        private readonly List<TargetSelection> missionTargets = new List<TargetSelection>();
        private int originIndex;
        private int destinationIndex;
        private int selectedTargetIndex;

        private string parkingAltitudeKm = "100";
        private string captureAltitudeKm = "100";
        private string searchYears = "5";
        private string earliestLeadDays = "0";
        private string tofMinDays = "";
        private string tofMaxDays = "";
        private string maxMissionYears = "8";
        private string maxFlybys = "3";
        private bool gravityAssistMode = true;
        private bool autoTargetOrder = true;
        private bool useMoonAssists = true;
        private bool showTrajectoryOverlay = true;
        private bool autoTof = true;
        private bool includeCapture = true;

        private readonly string[] qualityLabels = { "Быстро", "Нормально", "Точно" };
        private int qualityIndex = 1;

        private bool searchRunning;
        private bool cancelSearch;
        private float searchProgress;
        private string statusText = "Готово. Выбери тела и нажми «Искать окно».";
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
        private bool porkchopShowsMissionDuration;
        private GameObject trajectoryOverlayRoot;
        private TransferResult overlayResult;
        private Material trajectoryLineMaterial;
        private Material trajectoryMarkerMaterial;

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

            ClearTrajectoryOverlay();
            if (trajectoryLineMaterial != null)
            {
                Destroy(trajectoryLineMaterial);
                trajectoryLineMaterial = null;
            }
            if (trajectoryMarkerMaterial != null)
            {
                Destroy(trajectoryMarkerMaterial);
                trajectoryMarkerMaterial = null;
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
            ClearTrajectoryOverlay();
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
            ClearTrajectoryOverlay();
        }

        private void Update()
        {
            if (!windowVisible)
            {
                return;
            }

            if (showTrajectoryOverlay && bestResult != null && trajectoryOverlayRoot == null && IsMapOverlayScene())
            {
                BuildTrajectoryOverlay(bestResult);
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
                GUILayout.Label("Небесные тела ещё не загружены. Открой сохранение/сцену KSP и попробуй снова.");
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
            leftScroll = GUILayout.BeginScrollView(leftScroll, GUILayout.Width(295f), GUILayout.Height(545f));
            GUILayout.BeginVertical(GUILayout.Width(270f));

            GUILayout.Label("Параметры поиска", GUILayout.Height(22f));
            DrawBodySelector("Старт", ref originIndex);
            DrawTargetList();

            GUILayout.Space(6f);
            GUILayout.Label("Орбиты");
            DrawTextFieldRow("Начальная орбита, км", ref parkingAltitudeKm);
            DrawTextFieldRow("Финальная орбита, км", ref captureAltitudeKm);

            GUILayout.Space(6f);
            GUILayout.Label("Когда искать");
            DrawTextFieldRow("Не раньше чем через, дней", ref earliestLeadDays);
            DrawTextFieldRow("Искать ближайшие, лет", ref searchYears);

            GUILayout.Space(6f);
            gravityAssistMode = GUILayout.Toggle(gravityAssistMode, "Авто маршрут через flyby");
            GUI.enabled = gravityAssistMode;
            autoTargetOrder = GUILayout.Toggle(autoTargetOrder, "Авто порядок целей");
            useMoonAssists = GUILayout.Toggle(useMoonAssists, "Учитывать луны");
            bool previousOverlay = showTrajectoryOverlay;
            showTrajectoryOverlay = GUILayout.Toggle(showTrajectoryOverlay, "Показать траекторию");
            if (previousOverlay && !showTrajectoryOverlay)
            {
                ClearTrajectoryOverlay();
            }
            if (!previousOverlay && showTrajectoryOverlay && bestResult != null)
            {
                BuildTrajectoryOverlay(bestResult);
            }
            DrawTextFieldRow("Макс. время полёта, лет", ref maxMissionYears);
            DrawTextFieldRow("Макс. flyby-вставок", ref maxFlybys);
            GUI.enabled = true;

            GUILayout.Space(6f);
            GUI.enabled = !gravityAssistMode;
            GUILayout.BeginHorizontal();
            autoTof = GUILayout.Toggle(autoTof, "Авто время полёта", GUILayout.Width(170f));
            if (GUILayout.Button("?", GUILayout.Width(28f)))
            {
                statusText = "Авто TOF берёт диапазон около гомановского перелёта для выбранных орбит.";
            }
            GUILayout.EndHorizontal();

            GUI.enabled = !gravityAssistMode && !autoTof;
            DrawTextFieldRow("TOF min, дней", ref tofMinDays);
            DrawTextFieldRow("TOF max, дней", ref tofMaxDays);
            GUI.enabled = true;

            GUILayout.Label("Захват задаётся режимом каждой цели.");

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
                if (GUILayout.Button(gravityAssistMode ? "Искать маршрут" : "Искать окно", GUILayout.Height(34f)))
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
                "Прямой режим считает обычный Lambert-перелёт к первой цели. Авто маршрут посещает все цели, перебирает порядок, если включено, " +
                "и сравнивает прямые flyby с лунными assist-манёврами. Solver single-rev: резонансные/multi-rev окна могут быть не найдены.",
                GUILayout.Height(85f));

            GUILayout.EndVertical();
            GUILayout.EndScrollView();
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

            GUILayout.Space(6f);
            GUI.enabled = bestResult != null && !searchRunning;
            if (GUILayout.Button("Создать манёвр на активный аппарат", GUILayout.Height(30f)))
            {
                CreateFirstManeuverNode();
            }
            GUI.enabled = true;

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

        private void DrawTargetList()
        {
            EnsureTargetList();

            GUILayout.Space(4f);
            GUILayout.Label("Цели миссии", GUILayout.Height(20f));

            int removeIndex = -1;
            for (int i = 0; i < missionTargets.Count; i++)
            {
                DrawTargetRow(i, ref removeIndex);
            }

            if (removeIndex >= 0 && missionTargets.Count > 1)
            {
                missionTargets.RemoveAt(removeIndex);
                selectedTargetIndex = Mathf.Clamp(selectedTargetIndex, 0, missionTargets.Count - 1);
                SyncDestinationFromTargets();
            }

            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Добавить", GUILayout.Width(82f)) && missionTargets.Count < 5)
            {
                missionTargets.Add(new TargetSelection
                {
                    BodyIndex = GetNextDefaultTargetIndex(),
                    Mode = TargetVisitMode.Flyby
                });
                selectedTargetIndex = missionTargets.Count - 1;
                SyncDestinationFromTargets();
            }

            GUI.enabled = missionTargets.Count > 1;
            if (GUILayout.Button("Вверх", GUILayout.Width(62f)))
            {
                MoveTarget(selectedTargetIndex, -1);
            }
            if (GUILayout.Button("Вниз", GUILayout.Width(62f)))
            {
                MoveTarget(selectedTargetIndex, 1);
            }
            GUI.enabled = true;
            GUILayout.EndHorizontal();
        }

        private void DrawTargetRow(int targetIndex, ref int removeIndex)
        {
            TargetSelection target = missionTargets[targetIndex];
            bool selected = selectedTargetIndex == targetIndex;

            GUILayout.BeginVertical(GUI.skin.box, GUILayout.Width(260f));
            GUILayout.BeginHorizontal();
            if (GUILayout.Button(selected ? "●" : "○", GUILayout.Width(26f)))
            {
                selectedTargetIndex = targetIndex;
            }

            int bodyIndex = WrapIndex(target.BodyIndex, bodies.Count);
            if (GUILayout.Button("<", GUILayout.Width(28f)))
            {
                bodyIndex = WrapIndex(bodyIndex - 1, bodies.Count);
            }
            GUILayout.Label(GetBodyName(bodyIndex), GUILayout.Width(110f));
            if (GUILayout.Button(">", GUILayout.Width(28f)))
            {
                bodyIndex = WrapIndex(bodyIndex + 1, bodies.Count);
            }

            target.BodyIndex = bodyIndex;
            GUI.enabled = missionTargets.Count > 1;
            if (GUILayout.Button("X", GUILayout.Width(28f)))
            {
                removeIndex = targetIndex;
            }
            GUI.enabled = true;
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Space(28f);
            GUILayout.Label("Режим", GUILayout.Width(58f));
            if (GUILayout.Button(target.Mode == TargetVisitMode.Capture ? "Орбита" : "Пролёт", GUILayout.Width(92f)))
            {
                target.Mode = target.Mode == TargetVisitMode.Capture ? TargetVisitMode.Flyby : TargetVisitMode.Capture;
            }
            GUILayout.EndHorizontal();
            GUILayout.EndVertical();

            missionTargets[targetIndex] = target;
            SyncDestinationFromTargets();
        }

        private void MoveTarget(int index, int delta)
        {
            if (missionTargets.Count < 2)
            {
                return;
            }

            int from = Mathf.Clamp(index, 0, missionTargets.Count - 1);
            int to = Mathf.Clamp(from + delta, 0, missionTargets.Count - 1);
            if (from == to)
            {
                return;
            }

            TargetSelection target = missionTargets[from];
            missionTargets.RemoveAt(from);
            missionTargets.Insert(to, target);
            selectedTargetIndex = to;
            SyncDestinationFromTargets();
        }

        private int GetNextDefaultTargetIndex()
        {
            int start = missionTargets.Count > 0 ? missionTargets[missionTargets.Count - 1].BodyIndex + 1 : destinationIndex;
            for (int i = 0; i < bodies.Count; i++)
            {
                int candidate = WrapIndex(start + i, bodies.Count);
                if (candidate != WrapIndex(originIndex, bodies.Count))
                {
                    return candidate;
                }
            }

            return WrapIndex(destinationIndex, bodies.Count);
        }

        private void EnsureTargetList()
        {
            if (bodies.Count == 0)
            {
                return;
            }

            if (missionTargets.Count == 0)
            {
                missionTargets.Add(new TargetSelection
                {
                    BodyIndex = WrapIndex(destinationIndex, bodies.Count),
                    Mode = includeCapture ? TargetVisitMode.Capture : TargetVisitMode.Flyby
                });
            }

            for (int i = 0; i < missionTargets.Count; i++)
            {
                TargetSelection target = missionTargets[i];
                target.BodyIndex = WrapIndex(target.BodyIndex, bodies.Count);
                missionTargets[i] = target;
            }

            selectedTargetIndex = Mathf.Clamp(selectedTargetIndex, 0, missionTargets.Count - 1);
            SyncDestinationFromTargets();
        }

        private void SyncDestinationFromTargets()
        {
            if (missionTargets.Count > 0)
            {
                destinationIndex = WrapIndex(missionTargets[missionTargets.Count - 1].BodyIndex, bodies.Count);
            }
        }

        private List<MissionTarget> BuildMissionTargets()
        {
            EnsureTargetList();
            List<MissionTarget> targets = new List<MissionTarget>();
            for (int i = 0; i < missionTargets.Count; i++)
            {
                if (bodies.Count == 0)
                {
                    break;
                }

                TargetSelection selection = missionTargets[i];
                CelestialBody body = bodies[WrapIndex(selection.BodyIndex, bodies.Count)];
                targets.Add(new MissionTarget
                {
                    Body = body,
                    Mode = selection.Mode,
                    OriginalIndex = i
                });
            }

            return targets;
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

            GUI.Label(new Rect(plotRect.x + 8f, plotRect.y + 6f, 120f, 20f), porkchopShowsMissionDuration ? "Время миссии" : "TOF");
            GUI.Label(new Rect(plotRect.x + 190f, plotRect.y + plotRect.height - 24f, 220f, 20f), "Дата старта");

            if (bestGridI >= 0 && bestGridJ >= 0 && gridDepartSamples > 1 && gridTofSamples > 1)
            {
                float x = texRect.x + texRect.width * bestGridI / Math.Max(1f, gridDepartSamples - 1f);
                float y = texRect.y + texRect.height - texRect.height * bestGridJ / Math.Max(1f, gridTofSamples - 1f);
                GUI.Label(new Rect(x - 8f, y - 10f, 24f, 20f), "X");
            }

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

            if (bestResult.GravityRoute && bestResult.RouteBodies != null && bestResult.RouteLegs != null)
            {
                return BuildGravityRouteResultText(bestResult);
            }

            string longWayText = bestResult.LongWay ? "да" : "нет";
            string captureLine = bestResult.InsertionDV > 0.0
                ? "Δv захвата:       " + FormatDV(bestResult.InsertionDV) + "\n"
                : "Δv захвата:       нет обязательного захвата\n";

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

        private string BuildGravityRouteResultText(TransferResult result)
        {
            string text =
                "Авто маршрут с гравитационными манёврами\n" +
                result.RouteName + "\n" +
                "\n" +
                "Старт:              " + FormatUT(result.DepartureUT) + "\n" +
                "Финиш:              " + FormatUT(result.ArrivalUT) + "\n" +
                "Время полёта:       " + FormatDuration(result.TimeOfFlight) + "\n" +
                "\n" +
                "Итого Δv:           " + FormatDV(result.TotalDV) + "\n" +
                "Δv вылета:          " + FormatDV(result.EjectionDV) + "\n" +
                "Δv powered flyby:   " + FormatDV(result.FlybyDV) + "\n";

            if (result.InsertionDV > 0.0)
            {
                text += "Δv захвата:         " + FormatDV(result.InsertionDV) + "\n";
            }
            else
            {
                text += "Δv захвата:         нет обязательного захвата\n";
            }

            text +=
                "v∞ старта:          " + FormatSpeed(result.DepartureVInf) + "\n" +
                "v∞ прибытия:        " + FormatSpeed(result.ArrivalVInf) + "\n" +
                "\n" +
                "Узлы маршрута:\n";

            for (int i = 0; i < result.RouteBodies.Length; i++)
            {
                double ut = result.EncounterUTs != null && i < result.EncounterUTs.Length ? result.EncounterUTs[i] : result.DepartureUT;
                string mode = "";
                if (result.RouteNodes != null && i < result.RouteNodes.Length && result.RouteNodes[i].IsMandatory && i > 0)
                {
                    mode = result.RouteNodes[i].Mode == TargetVisitMode.Capture ? " / захват" : " / пролёт";
                }
                else if (i > 0)
                {
                    mode = " / assist";
                }

                text += "  " + (i + 1) + ". " + result.RouteBodies[i].bodyName + mode + " — " + FormatUT(ut) + "\n";
            }

            text += "\nУчастки:\n";
            for (int i = 0; i < result.RouteLegs.Length; i++)
            {
                RouteLegResult leg = result.RouteLegs[i];
                text +=
                    "  " + leg.From.bodyName + " -> " + leg.To.bodyName +
                    " / " + FormatDuration(leg.TimeOfFlight) +
                    " / v∞ " + FormatSpeed(leg.DepartureVInf) + " -> " + FormatSpeed(leg.ArrivalVInf);

                if (i < result.RouteLegs.Length - 1)
                {
                    string assistName = string.IsNullOrEmpty(leg.FlybyAssistName) ? leg.To.bodyName : leg.FlybyAssistName;
                    text +=
                        " / flyby Δv " + FormatDV(leg.FlybyPoweredDV) +
                        " / assist " + assistName +
                        " / поворот " + FormatAngle(leg.RequiredTurnDeg) +
                        " из " + FormatAngle(leg.FreeTurnDeg);
                }

                if (i < result.RouteLegs.Length - 1 && IsFinite(leg.PlannedFlybyPeriapsisRadius) && leg.PlannedFlybyPeriapsisRadius > 0.0)
                {
                    text += " / Pe " + FormatDistance(Math.Max(0.0, leg.PlannedFlybyPeriapsisRadius - leg.To.Radius));
                }
                else if (IsFinite(leg.PlannedCapturePeriapsisRadius) && leg.PlannedCapturePeriapsisRadius > 0.0)
                {
                    text += " / фин. Pe " + FormatDistance(Math.Max(0.0, leg.PlannedCapturePeriapsisRadius - leg.To.Radius));
                }

                if (leg.CaptureDV > 0.0 || !string.IsNullOrEmpty(leg.CaptureAssistName))
                {
                    string captureAssist = string.IsNullOrEmpty(leg.CaptureAssistName) ? leg.To.bodyName : leg.CaptureAssistName;
                    text +=
                        " / захват " + FormatDV(leg.CaptureDV) +
                        " / braking " + captureAssist +
                        " / v∞ после " + FormatSpeed(leg.CapturePostAssistVInf);
                    if (leg.CapturePoweredDV > 0.0)
                    {
                        text += " / powered " + FormatDV(leg.CapturePoweredDV);
                    }
                }

                text += "\n";
            }

            text +=
                "\nПримечание: это поисковая оценка patched-conics. Бесплатный flyby засчитывается, если планета может повернуть v∞ на нужный угол на безопасной высоте пролёта. " +
                "Если не может, добавляется оценочный powered-flyby штраф Δv. Точная миссия всё равно требует ручной донастройки maneuver nodes.";

            return text;
        }

        private void BeginSearch()
        {
            RefreshBodies(false);
            EnsureTargetList();
            ClearTrajectoryOverlay();

            if (bodies.Count < 2)
            {
                statusText = "Не удалось найти тела в текущей игре.";
                return;
            }

            CelestialBody origin = bodies[WrapIndex(originIndex, bodies.Count)];
            Vessel planningVessel = GetActivePlanningVessel();
            bool useActiveVesselStart = planningVessel != null &&
                planningVessel.orbit != null &&
                planningVessel.orbit.referenceBody != null &&
                planningVessel.orbit.referenceBody.orbit != null;
            if (useActiveVesselStart)
            {
                origin = planningVessel.orbit.referenceBody;
                int activeOriginIndex = bodies.IndexOf(origin);
                if (activeOriginIndex >= 0)
                {
                    originIndex = activeOriginIndex;
                }
            }
            List<MissionTarget> missionTargetVisits = BuildMissionTargets();
            if (missionTargetVisits.Count == 0)
            {
                statusText = "Добавь хотя бы одну цель миссии.";
                return;
            }

            int finalTargetIndex = gravityAssistMode
                ? GetRouteFinalTargetIndex(missionTargetVisits, true)
                : RoutePlanningRules.GetDirectTargetIndex(missionTargetVisits.Count);
            CelestialBody destination = missionTargetVisits[finalTargetIndex].Body;

            if (origin == destination)
            {
                statusText = "Финальная цель должна отличаться от старта.";
                return;
            }

            if (origin.orbit == null || origin.orbit.referenceBody == null)
            {
                statusText = "Стартовое тело не имеет орбиты. Обычно это звезда/центр системы.";
                return;
            }

            for (int targetIndex = 0; targetIndex < missionTargetVisits.Count; targetIndex++)
            {
                CelestialBody targetBody = missionTargetVisits[targetIndex].Body;
                if (targetBody == null || targetBody.orbit == null || targetBody.orbit.referenceBody == null)
                {
                    statusText = "Одна из выбранных целей не имеет орбиты. Обычно это звезда/центр системы.";
                    return;
                }

            }

            double parkingAlt = ParseDouble(parkingAltitudeKm, 100.0) * 1000.0;
            double captureAlt = ParseDouble(captureAltitudeKm, 100.0) * 1000.0;
            string validationError;
            if (!ValidateMissionLegs(origin, missionTargetVisits, gravityAssistMode, autoTargetOrder, out validationError) ||
                !TryValidateSearchAltitudes(origin, missionTargetVisits, gravityAssistMode, parkingAlt, captureAlt, out validationError))
            {
                statusText = validationError;
                return;
            }

            double lead = Math.Max(0.0, ParseDouble(earliestLeadDays, 0.0)) * DaySeconds();
            double years = Math.Max(0.02, ParseDouble(searchYears, 5.0));
            double searchSpan = years * YearSeconds();
            double startUT = Planetarium.GetUniversalTime() + lead;

            if (gravityAssistMode)
            {
                int routeDepartSamples;
                int legSamples;
                int beamWidth;
                GetGravityAssistQualitySamples(out routeDepartSamples, out legSamples, out beamWidth);
                routeDepartSamples = AdjustGravityAssistDepartSamples(routeDepartSamples, searchSpan);

                int assistLimit = Math.Max(0, Math.Min(4, (int)Math.Round(ParseDouble(maxFlybys, 3.0))));
                double maxFlight = Math.Max(DaySeconds() * 30.0, ParseDouble(maxMissionYears, 8.0) * YearSeconds());

                bestResult = null;
                dvGrid = null;
                bestGridI = -1;
                bestGridJ = -1;
                porkchopShowsMissionDuration = true;
                searchProgress = 0f;
                cancelSearch = false;
                searchRunning = true;
                statusText = "Начинаю поиск маршрута с гравитационными манёврами...";

                if (porkchopTexture != null)
                {
                    Destroy(porkchopTexture);
                    porkchopTexture = null;
                }

                GravityRouteSearchRequest routeRequest = new GravityRouteSearchRequest
                {
                    Origin = origin,
                    Destination = destination,
                    Targets = missionTargetVisits,
                    AutoTargetOrder = autoTargetOrder,
                    ParkingAltitude = Math.Max(0.0, parkingAlt),
                    CaptureAltitude = Math.Max(0.0, captureAlt),
                    StartUT = startUT,
                    SearchSpan = searchSpan,
                    MaxMissionTime = maxFlight,
                    MaxFlybys = assistLimit,
                    DepartSamples = routeDepartSamples,
                    LegSamples = legSamples,
                    BeamWidth = beamWidth,
                    IncludeCapture = true,
                    UseMoonAssists = useMoonAssists,
                    UseActiveVesselStart = useActiveVesselStart,
                    ActiveVesselOrbit = useActiveVesselStart ? planningVessel.orbit : null,
                    ActiveVesselName = useActiveVesselStart ? planningVessel.vesselName : null
                };

                StartCoroutine(SearchGravityRouteCoroutine(routeRequest));
                return;
            }

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
            porkchopShowsMissionDuration = false;
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
                IncludeCapture = missionTargetVisits[RoutePlanningRules.GetDirectTargetIndex(missionTargetVisits.Count)].Mode == TargetVisitMode.Capture,
                UseActiveVesselStart = useActiveVesselStart,
                ActiveVesselOrbit = useActiveVesselStart ? planningVessel.orbit : null,
                ActiveVesselName = useActiveVesselStart ? planningVessel.vesselName : null
            };

            StartCoroutine(SearchCoroutine(request));
        }

        private static Vessel GetActivePlanningVessel()
        {
            if (HighLogic.LoadedScene != GameScenes.FLIGHT)
            {
                return null;
            }

            Vessel vessel = FlightGlobals.ActiveVessel;
            if (vessel == null || vessel.orbit == null || vessel.orbit.referenceBody == null)
            {
                return null;
            }

            return vessel;
        }

        private static bool ValidateMissionLegs(CelestialBody origin, List<MissionTarget> targets, bool gravityMode, bool autoOrder, out string error)
        {
            error = null;
            if (origin == null || targets == null || targets.Count == 0)
            {
                error = "Добавь хотя бы одну цель миссии.";
                return false;
            }

            if (!gravityMode)
            {
                int directTargetIndex = RoutePlanningRules.GetDirectTargetIndex(targets.Count);
                return TryValidateLegGeometry(origin, targets[directTargetIndex].Body, out error);
            }

            if (autoOrder)
            {
                return true;
            }

            CelestialBody previous = origin;
            for (int i = 0; i < targets.Count; i++)
            {
                if (!TryValidateLegGeometry(previous, targets[i].Body, out error))
                {
                    return false;
                }

                previous = targets[i].Body;
            }

            return true;
        }

        private static bool TryValidateLegGeometry(CelestialBody from, CelestialBody to, out string error)
        {
            error = null;
            LegGeometry geometry;
            if (!TryGetLegGeometry(from, to, out geometry))
            {
                string fromName = from != null ? from.bodyName : "?";
                string toName = to != null ? to.bodyName : "?";
                error = "Не удалось подобрать patched-conics фрейм для участка " + fromName + " -> " + toName + ".";
                return false;
            }

            return true;
        }

        private static bool TryValidateSearchAltitudes(
            CelestialBody origin,
            List<MissionTarget> targets,
            bool gravityMode,
            double parkingAltitude,
            double captureAltitude,
            out string error)
        {
            error = null;
            if (!TryValidateOrbitAltitude(origin, parkingAltitude, "начальной орбиты", out error))
            {
                return false;
            }

            if (!gravityMode)
            {
                int directTargetIndex = RoutePlanningRules.GetDirectTargetIndex(targets != null ? targets.Count : 0);
                if (directTargetIndex >= 0 && targets[directTargetIndex].Mode == TargetVisitMode.Capture)
                {
                    return TryValidateOrbitAltitude(targets[directTargetIndex].Body, captureAltitude, "финальной орбиты", out error);
                }

                return true;
            }

            for (int i = 0; targets != null && i < targets.Count; i++)
            {
                MissionTarget target = targets[i];
                if (target != null && target.Mode == TargetVisitMode.Capture &&
                    !TryValidateOrbitAltitude(target.Body, captureAltitude, "орбиты захвата", out error))
                {
                    return false;
                }
            }

            return true;
        }

        private static bool TryValidateOrbitAltitude(CelestialBody body, double altitude, string label, out string error)
        {
            error = null;
            if (body == null)
            {
                error = "Тело для проверки " + label + " не задано.";
                return false;
            }

            double minimum = GetMinimumSafeOrbitAltitude(body);
            if (!OrbitSafety.IsAltitudeSafe(altitude, minimum))
            {
                error = "Высота " + label + " у " + body.bodyName + " слишком мала. Минимум: " + FormatDistance(minimum) + ".";
                return false;
            }

            return true;
        }

        private static double GetMinimumSafeOrbitAltitude(CelestialBody body)
        {
            if (body == null)
            {
                return OrbitSafety.DefaultMinimumAltitude;
            }

            double atmosphereDepth = body.atmosphere ? body.atmosphereDepth : 0.0;
            return OrbitSafety.MinimumSafeAltitude(body.Radius, body.atmosphere, atmosphereDepth, GetTerrainRadiusMax(body));
        }

        private static double GetTerrainRadiusMax(CelestialBody body)
        {
            if (body == null || body.pqsController == null)
            {
                return double.NaN;
            }

            double radiusMax = body.pqsController.radiusMax;
            return IsFinite(radiusMax) && radiusMax > 0.0 ? radiusMax : double.NaN;
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
            System.Diagnostics.Stopwatch frameTimer = System.Diagnostics.Stopwatch.StartNew();

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
                    if (TryEvaluateTransfer(request, departUT, tof, false, out result))
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
                    if (frameTimer.Elapsed.TotalSeconds >= SearchFrameBudgetSeconds)
                    {
                        searchProgress = 0.82f * evalCount / totalCount;
                        yield return null;
                        frameTimer.Reset();
                        frameTimer.Start();
                    }
                }
            }

            if (!cancelSearch && best != null)
            {
                best = RefineBest(request, best, 3);
                if (best != null)
                {
                    bestResult = best;
                    statusText = "Готово. Лучшее окно найдено: " + FormatDV(best.TotalDV) + "." + SingleRevWarning;
                    if (showTrajectoryOverlay)
                    {
                        BuildTrajectoryOverlay(bestResult);
                    }
                }
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

        private IEnumerator SearchGravityRouteCoroutine(GravityRouteSearchRequest request)
        {
            List<RouteTemplate> routeTemplates = BuildRouteTemplates(request);
            if (routeTemplates.Count == 0)
            {
                statusText = "Не удалось собрать список маршрутов для этих тел.";
                searchProgress = 1f;
                searchRunning = false;
                yield break;
            }

            TransferResult best = null;
            RouteTemplate bestTemplate = null;
            double bestDepartUT = 0.0;
            List<RouteSearchSeed> topSeeds = new List<RouteSearchSeed>();
            double bestScore = double.PositiveInfinity;
            int evalCount = 0;
            int totalCount = Math.Max(1, routeTemplates.Count * request.DepartSamples);
            System.Diagnostics.Stopwatch frameTimer = System.Diagnostics.Stopwatch.StartNew();
            gridDepartSamples = request.DepartSamples;
            gridTofSamples = GetGravityPorkchopRows();
            gridStartUT = request.StartUT;
            gridSearchSpan = request.SearchSpan;
            gridTofMin = 0.0;
            gridTofMax = request.MaxMissionTime;
            porkchopShowsMissionDuration = true;
            dvGrid = new double[gridDepartSamples, gridTofSamples];
            for (int i = 0; i < gridDepartSamples; i++)
            {
                for (int j = 0; j < gridTofSamples; j++)
                {
                    dvGrid[i, j] = double.NaN;
                }
            }

            statusText = "Маршрутов: " + routeTemplates.Count + ". Ищу цепочку flyby...";

            for (int i = 0; i < request.DepartSamples; i++)
            {
                if (cancelSearch)
                {
                    break;
                }

                double departUT = request.StartUT + request.SearchSpan * i / Math.Max(1, request.DepartSamples - 1);

                for (int routeIndex = 0; routeIndex < routeTemplates.Count; routeIndex++)
                {
                    if (cancelSearch)
                    {
                        break;
                    }

                    TransferResult candidate;
                    if (TrySearchRouteTemplate(request, routeTemplates[routeIndex], departUT, false, bestScore, out candidate))
                    {
                        int porkchopJ = RecordGravityPorkchopCandidate(candidate, i, request);
                        AddRouteSearchSeed(topSeeds, candidate, routeTemplates[routeIndex], departUT, porkchopJ, 8);
                        if (candidate.TotalDV < bestScore)
                        {
                            bestScore = candidate.TotalDV;
                            best = candidate;
                            bestTemplate = routeTemplates[routeIndex];
                            bestDepartUT = departUT;
                            bestGridI = i;
                            bestGridJ = porkchopJ;
                            statusText = "Лучший маршрут: " + candidate.RouteName + " / " + FormatDV(candidate.TotalDV);
                        }
                    }

                    evalCount++;
                    if (frameTimer.Elapsed.TotalSeconds >= SearchFrameBudgetSeconds)
                    {
                        searchProgress = 0.96f * evalCount / totalCount;
                        yield return null;
                        frameTimer.Reset();
                        frameTimer.Start();
                    }
                }
            }

            if (!cancelSearch && best != null)
            {
                TransferResult refinedBest = null;
                double departStep = request.SearchSpan / Math.Max(1, request.DepartSamples - 1);
                double[] departOffsets = new[] { 0.0, -departStep * 0.45, departStep * 0.45 };
                if (topSeeds.Count == 0 && bestTemplate != null)
                {
                    AddRouteSearchSeed(topSeeds, best, bestTemplate, bestDepartUT, bestGridJ, 1);
                }

                for (int seedIndex = 0; seedIndex < topSeeds.Count; seedIndex++)
                {
                    RouteSearchSeed seed = topSeeds[seedIndex];
                    for (int refineIndex = 0; refineIndex < departOffsets.Length; refineIndex++)
                    {
                        double refinedDepartUT = seed.DepartUT + departOffsets[refineIndex];
                        if (refinedDepartUT < request.StartUT || refinedDepartUT > request.StartUT + request.SearchSpan)
                        {
                            continue;
                        }

                        TransferResult exact;
                        if (TrySearchRouteTemplate(request, seed.Template, refinedDepartUT, true, double.PositiveInfinity, out exact, seed.Result) &&
                            (refinedBest == null || exact.TotalDV < refinedBest.TotalDV))
                        {
                            refinedBest = exact;
                        }

                        searchProgress = 0.98f;
                        yield return null;
                    }
                }

                if (refinedBest == null)
                {
                    best = null;
                    bestResult = null;
                    statusText = "Маршрут найден в грубом поиске, но точная геометрия SOI/flyby его не подтвердила. Увеличь время полёта или попробуй «Точно».";
                }
                else
                {
                    best = refinedBest;
                }

                if (best != null)
                {
                    bestResult = best;
                    statusText = "Готово. Лучший маршрут: " + best.RouteName + " / " + FormatDV(best.TotalDV) + "." + SingleRevWarning;
                    if (showTrajectoryOverlay)
                    {
                        BuildTrajectoryOverlay(bestResult);
                    }
                }
            }
            else if (cancelSearch)
            {
                statusText = "Поиск остановлен.";
            }
            else
            {
                statusText = "Маршрут не найден. Увеличь максимальное время полёта, годы поиска или число пролётов.";
            }

            BuildPorkchopTexture();
            searchProgress = 1f;
            searchRunning = false;
        }

        private int GetGravityPorkchopRows()
        {
            if (qualityIndex <= 0)
            {
                return 36;
            }
            if (qualityIndex == 1)
            {
                return 48;
            }

            return 64;
        }

        private int RecordGravityPorkchopCandidate(TransferResult candidate, int departIndex, GravityRouteSearchRequest request)
        {
            if (candidate == null || dvGrid == null || gridTofSamples <= 0)
            {
                return -1;
            }

            double missionFraction = request.MaxMissionTime > 0.0 ? candidate.TimeOfFlight / request.MaxMissionTime : 0.0;
            int j = Mathf.Clamp((int)Math.Round(Clamp01(missionFraction) * Math.Max(1, gridTofSamples - 1)), 0, Math.Max(0, gridTofSamples - 1));
            double current = dvGrid[departIndex, j];
            if (!IsFinite(current) || candidate.TotalDV < current)
            {
                dvGrid[departIndex, j] = candidate.TotalDV;
            }

            return j;
        }

        private static void AddRouteSearchSeed(List<RouteSearchSeed> seeds, TransferResult result, RouteTemplate template, double departUT, int porkchopJ, int maxCount)
        {
            if (seeds == null || result == null || template == null || maxCount <= 0)
            {
                return;
            }

            seeds.Add(new RouteSearchSeed
            {
                Result = result,
                Template = template,
                DepartUT = departUT,
                PorkchopJ = porkchopJ
            });

            seeds.Sort(delegate (RouteSearchSeed a, RouteSearchSeed b)
            {
                double av = a != null && a.Result != null ? a.Result.TotalDV : double.PositiveInfinity;
                double bv = b != null && b.Result != null ? b.Result.TotalDV : double.PositiveInfinity;
                return av.CompareTo(bv);
            });

            if (seeds.Count > maxCount)
            {
                seeds.RemoveRange(maxCount, seeds.Count - maxCount);
            }
        }

        private List<RouteTemplate> BuildRouteTemplates(GravityRouteSearchRequest request)
        {
            List<RouteTemplate> routes = new List<RouteTemplate>(Math.Min(MaxRouteTemplateCount(), 64));
            List<MissionTarget[]> targetOrders = BuildTargetOrders(request.Targets, request.AutoTargetOrder);
            List<CelestialBody> flybyCandidates = GetFlybyCandidates(request.Origin, request.Targets);

            for (int orderIndex = 0; orderIndex < targetOrders.Count; orderIndex++)
            {
                MissionTarget[] orderedTargets = targetOrders[orderIndex];
                List<RouteNode> nodes = new List<RouteNode>(orderedTargets.Length + request.MaxFlybys + 1);
                nodes.Add(new RouteNode
                {
                    Body = request.Origin,
                    Mode = TargetVisitMode.Flyby,
                    IsMandatory = true,
                    TargetIndex = -1
                });

                for (int i = 0; i < orderedTargets.Length; i++)
                {
                    nodes.Add(new RouteNode
                    {
                        Body = orderedTargets[i].Body,
                        Mode = orderedTargets[i].Mode,
                        IsMandatory = true,
                        TargetIndex = orderedTargets[i].OriginalIndex
                    });
                }

                AddRouteTemplate(routes, nodes, orderedTargets);
                for (int flybyCount = 1; flybyCount <= request.MaxFlybys && routes.Count < MaxRouteTemplateCount(); flybyCount++)
                {
                    BuildRouteTemplatesRecursive(request, flybyCandidates, flybyCount, nodes, orderedTargets, routes);
                }
            }

            return routes;
        }

        private List<MissionTarget[]> BuildTargetOrders(List<MissionTarget> targets, bool autoOrder)
        {
            List<MissionTarget[]> orders = new List<MissionTarget[]>();
            int finalTargetIndex = GetRouteFinalTargetIndex(targets, true);
            if (!autoOrder || targets.Count <= 1)
            {
                orders.Add(BuildTargetOrderWithFixedFinal(targets, finalTargetIndex));
                return orders;
            }

            MissionTarget finalTarget = targets[finalTargetIndex];
            List<MissionTarget> work = new List<MissionTarget>();
            for (int i = 0; i < targets.Count; i++)
            {
                if (i != finalTargetIndex)
                {
                    work.Add(targets[i]);
                }
            }

            BuildTargetOrdersRecursive(work, 0, orders, finalTarget);
            return orders;
        }

        private static MissionTarget[] BuildTargetOrderWithFixedFinal(List<MissionTarget> targets, int finalTargetIndex)
        {
            if (targets == null || targets.Count == 0)
            {
                return new MissionTarget[0];
            }

            if (finalTargetIndex < 0 || finalTargetIndex >= targets.Count)
            {
                finalTargetIndex = targets.Count - 1;
            }

            MissionTarget[] order = new MissionTarget[targets.Count];
            int writeIndex = 0;
            for (int i = 0; i < targets.Count; i++)
            {
                if (i == finalTargetIndex)
                {
                    continue;
                }

                order[writeIndex++] = targets[i];
            }
            order[order.Length - 1] = targets[finalTargetIndex];
            return order;
        }

        private static int GetRouteFinalTargetIndex(List<MissionTarget> targets, bool preferCaptureFinal)
        {
            return RoutePlanningRules.GetFinalTargetIndex(
                targets != null ? targets.Count : 0,
                preferCaptureFinal,
                delegate (int i)
                {
                    return targets[i] != null && targets[i].Mode == TargetVisitMode.Capture;
                });
        }

        private void BuildTargetOrdersRecursive(List<MissionTarget> work, int index, List<MissionTarget[]> orders, MissionTarget fixedFinalTarget)
        {
            if (orders.Count >= MaxTargetOrderCount())
            {
                return;
            }

            if (index >= work.Count)
            {
                MissionTarget[] order = new MissionTarget[work.Count + 1];
                for (int i = 0; i < work.Count; i++)
                {
                    order[i] = work[i];
                }
                order[order.Length - 1] = fixedFinalTarget;
                orders.Add(order);
                return;
            }

            for (int i = index; i < work.Count; i++)
            {
                MissionTarget tmp = work[index];
                work[index] = work[i];
                work[i] = tmp;

                BuildTargetOrdersRecursive(work, index + 1, orders, fixedFinalTarget);

                tmp = work[index];
                work[index] = work[i];
                work[i] = tmp;
            }
        }

        private void BuildRouteTemplatesRecursive(
            GravityRouteSearchRequest request,
            List<CelestialBody> flybyCandidates,
            int flybysLeft,
            List<RouteNode> currentNodes,
            MissionTarget[] orderedTargets,
            List<RouteTemplate> routes)
        {
            if (routes.Count >= MaxRouteTemplateCount())
            {
                return;
            }

            if (flybysLeft == 0)
            {
                AddRouteTemplate(routes, currentNodes, orderedTargets);
                return;
            }

            for (int insertIndex = 1; insertIndex < currentNodes.Count && routes.Count < MaxRouteTemplateCount(); insertIndex++)
            {
                for (int candidateIndex = 0; candidateIndex < flybyCandidates.Count && routes.Count < MaxRouteTemplateCount(); candidateIndex++)
                {
                    CelestialBody body = flybyCandidates[candidateIndex];
                    RouteNode node = new RouteNode
                    {
                        Body = body,
                        Mode = TargetVisitMode.Flyby,
                        IsMandatory = false,
                        TargetIndex = -1
                    };

                    if (!CanInsertFlyby(currentNodes, insertIndex, body))
                    {
                        continue;
                    }

                    currentNodes.Insert(insertIndex, node);
                    BuildRouteTemplatesRecursive(request, flybyCandidates, flybysLeft - 1, currentNodes, orderedTargets, routes);
                    currentNodes.RemoveAt(insertIndex);
                }
            }
        }

        private static bool CanInsertFlyby(List<RouteNode> nodes, int insertIndex, CelestialBody body)
        {
            if ((insertIndex > 0 && nodes[insertIndex - 1].Body == body) ||
                (insertIndex < nodes.Count && nodes[insertIndex].Body == body))
            {
                return false;
            }

            int sameCount = 0;
            for (int i = insertIndex - 1; i >= 0; i--)
            {
                if (nodes[i].Body != body)
                {
                    break;
                }
                sameCount++;
            }
            for (int i = insertIndex; i < nodes.Count; i++)
            {
                if (nodes[i].Body != body)
                {
                    break;
                }
                sameCount++;
            }

            return sameCount < 2;
        }

        private static void AddRouteTemplate(List<RouteTemplate> routes, List<RouteNode> nodes, MissionTarget[] orderedTargets)
        {
            RouteNode[] copy = nodes.ToArray();
            int failedPairIndex;
            if (!RoutePlanningRules.TryValidateAdjacentPairs(copy.Length, delegate (int pairIndex)
                {
                    LegGeometry geometry;
                    return copy[pairIndex].Body != null &&
                        copy[pairIndex + 1].Body != null &&
                        TryGetLegGeometry(copy[pairIndex].Body, copy[pairIndex + 1].Body, out geometry);
                }, out failedPairIndex))
            {
                return;
            }

            MissionTarget[] targetCopy = new MissionTarget[orderedTargets.Length];
            Array.Copy(orderedTargets, targetCopy, orderedTargets.Length);
            routes.Add(new RouteTemplate
            {
                Nodes = copy,
                OrderedTargets = targetCopy
            });
        }

        private List<CelestialBody> GetFlybyCandidates(CelestialBody origin, List<MissionTarget> targets)
        {
            CelestialBody central = GetRouteSearchCentral(origin, targets);
            if (central == null)
            {
                central = origin.orbit.referenceBody;
            }

            double now = Planetarium.GetUniversalTime();
            double minSma = EstimateOrbitRadiusAroundCentral(origin, central, now);
            double maxSma = minSma;
            for (int i = 0; i < targets.Count; i++)
            {
                double sma = EstimateOrbitRadiusAroundCentral(targets[i].Body, central, now);
                minSma = Math.Min(minSma, sma);
                maxSma = Math.Max(maxSma, sma);
            }

            double lower = Math.Max(1.0, minSma * 0.42);
            double upper = Math.Max(lower * 1.1, maxSma * 1.35);

            List<CandidateBody> candidates = new List<CandidateBody>(bodies.Count);
            for (int i = 0; i < bodies.Count; i++)
            {
                CelestialBody body = bodies[i];
                if (body == null || body.orbit == null || body.orbit.referenceBody != central)
                {
                    continue;
                }

                double sma = EstimateOrbitRadiusAroundCentral(body, central, now);
                if (!IsFinite(sma) || sma <= 0.0)
                {
                    continue;
                }

                double score = 0.0;
                if (sma < lower)
                {
                    score = Math.Abs(Math.Log(lower / sma));
                }
                else if (sma > upper)
                {
                    score = Math.Abs(Math.Log(sma / upper));
                }

                if (body == origin)
                {
                    score -= 0.25;
                }

                candidates.Add(new CandidateBody { Body = body, Score = score, SemiMajorAxis = sma });
            }

            candidates.Sort(delegate (CandidateBody a, CandidateBody b)
            {
                int scoreCompare = a.Score.CompareTo(b.Score);
                if (scoreCompare != 0) return scoreCompare;
                return a.SemiMajorAxis.CompareTo(b.SemiMajorAxis);
            });

            List<CelestialBody> result = new List<CelestialBody>();
            int candidateLimit = qualityIndex <= 0 ? 6 : (qualityIndex == 1 ? 8 : 12);
            int limit = Math.Min(candidateLimit, candidates.Count);
            for (int i = 0; i < limit; i++)
            {
                AddBodyIfMissing(result, candidates[i].Body);
            }

            AddBodyIfMissing(result, origin);
            for (int i = 0; targets != null && i < targets.Count; i++)
            {
                if (targets[i] != null)
                {
                    AddBodyIfMissing(result, targets[i].Body);
                }
            }

            return result;
        }

        private static void AddBodyIfMissing(List<CelestialBody> bodiesList, CelestialBody body)
        {
            if (bodiesList == null || body == null || bodiesList.Contains(body))
            {
                return;
            }

            bodiesList.Add(body);
        }

        private bool TrySearchRouteTemplate(GravityRouteSearchRequest request, RouteTemplate template, double departUT, bool exactGeometry, double pruneScore, out TransferResult result, TransferResult durationSeed = null)
        {
            result = null;
            RouteNode[] route = template.Nodes;
            bool allowPrune = !exactGeometry && IsFinite(pruneScore) && pruneScore > 0.0;

            List<RouteState> beam = new List<RouteState>();
            RouteState start = new RouteState
            {
                Time = departUT,
                Cost = 0.0,
                EjectionDV = 0.0,
                InsertionDV = 0.0,
                FlybyDV = 0.0,
                IncomingVInf = new Vector3d(0.0, 0.0, 0.0),
                Times = new List<double> { departUT },
                Legs = new List<RouteLegResult>()
            };
            beam.Add(start);

            for (int legIndex = 0; legIndex < route.Length - 1; legIndex++)
            {
                RouteNode fromNode = route[legIndex];
                RouteNode toNode = route[legIndex + 1];
                bool isFirstLeg = legIndex == 0;
                List<RouteState> nextBeam = new List<RouteState>();

                for (int stateIndex = 0; stateIndex < beam.Count; stateIndex++)
                {
                    RouteState state = beam[stateIndex];
                    double minRemaining = EstimateMinimumRemainingRouteTime(route, legIndex + 1);
                    double maxArrivalUT = departUT + request.MaxMissionTime - minRemaining;
                    if (maxArrivalUT <= state.Time + DaySeconds())
                    {
                        continue;
                    }

                    int legSampleCount = exactGeometry ? Math.Min(request.LegSamples + 2, 12) : request.LegSamples;
                    double seedDuration = GetSeedLegDuration(durationSeed, legIndex);
                    List<double> durations = GenerateLegDurations(fromNode.Body, toNode.Body, state.Time, maxArrivalUT, legSampleCount, seedDuration);
                    for (int durationIndex = 0; durationIndex < durations.Count; durationIndex++)
                    {
                        double duration = durations[durationIndex];
                        double arrivalUT = state.Time + duration;
                        if (arrivalUT > maxArrivalUT)
                        {
                            continue;
                        }

                        List<RouteLegResult> legOptions = isFirstLeg && request.UseActiveVesselStart
                            ? EvaluateActiveVesselFirstLeg(request.ActiveVesselOrbit, fromNode.Body, toNode.Body, state.Time, duration, request.StartUT, request.ParkingAltitude, request.CaptureAltitude, exactGeometry ? 9 : 5)
                            : EvaluateLambertLeg(fromNode.Body, toNode.Body, state.Time, duration, request.ParkingAltitude, request.CaptureAltitude);
                        for (int optionIndex = 0; optionIndex < legOptions.Count; optionIndex++)
                        {
                            RouteLegResult leg = legOptions[optionIndex];
                            if (leg.ArrivalUT > maxArrivalUT)
                            {
                                continue;
                            }

                            CaptureEvaluation appliedCapture = null;
                            if (toNode.IsMandatory && toNode.Mode == TargetVisitMode.Capture)
                            {
                                RouteLegResult captureLeg;
                                if (!TryApplyCaptureToLeg(leg, request.CaptureAltitude, exactGeometry, request.UseMoonAssists, out captureLeg, out appliedCapture))
                                {
                                    continue;
                                }

                                leg = captureLeg;
                            }

                            double ejectionDV = state.EjectionDV;
                            double insertionDV = state.InsertionDV;
                            double flybyDV = state.FlybyDV;
                            double addCost = 0.0;
                            FlybyEvaluation appliedFlyby = null;

                            if (isFirstLeg || (fromNode.IsMandatory && fromNode.Mode == TargetVisitMode.Capture))
                            {
                                double ejection = GetRouteLegEjectionDV(leg, request.ParkingAltitude);
                                addCost += ejection;
                                ejectionDV += ejection;
                            }
                            else
                            {
                                RouteLegResult previousLeg = state.Legs.Count > 0 ? state.Legs[state.Legs.Count - 1] : null;
                                if (CanConnectLegsByFreeFlyby(previousLeg, leg))
                                {
                                    FlybyEvaluation flyby = EvaluateBestFlyby(fromNode.Body, state.IncomingVInf, leg.DepartureVInfVector, state.Time, exactGeometry && request.UseMoonAssists);
                                    if (!flyby.IsValid)
                                    {
                                        continue;
                                    }

                                    addCost += flyby.PoweredDV;
                                    flybyDV += flyby.PoweredDV;
                                    appliedFlyby = flyby;
                                }
                                else
                                {
                                    double boundaryCapture = CircularOrbitToHyperbolaDV(fromNode.Body, request.ParkingAltitude, state.IncomingVInf.magnitude);
                                    double boundaryEjection = GetRouteLegEjectionDV(leg, request.ParkingAltitude);
                                    if (!IsFinite(boundaryCapture) || !IsFinite(boundaryEjection))
                                    {
                                        continue;
                                    }

                                    addCost += boundaryCapture + boundaryEjection;
                                    insertionDV += boundaryCapture;
                                    ejectionDV += boundaryEjection;
                                }
                            }

                            if (appliedCapture != null)
                            {
                                addCost += appliedCapture.TotalDV;
                                insertionDV += appliedCapture.CaptureDV;
                                flybyDV += appliedCapture.PoweredDV;
                            }

                            if (allowPrune && state.Cost + addCost > pruneScore * 1.20)
                            {
                                continue;
                            }

                            RouteState next = state.Clone();
                            next.Time = leg.ArrivalUT;
                            next.Cost = state.Cost + addCost;
                            next.EjectionDV = ejectionDV;
                            next.InsertionDV = insertionDV;
                            next.FlybyDV = flybyDV;
                            next.IncomingVInf = leg.ArrivalVInfVector;
                            if (isFirstLeg && next.Times.Count > 0)
                            {
                                next.Times[0] = leg.DepartUT;
                            }
                            next.Times.Add(leg.ArrivalUT);
                            if (appliedFlyby != null && next.Legs.Count > 0)
                            {
                                RouteLegResult previousLeg = next.Legs[next.Legs.Count - 1];
                                RouteLegResult refinedPreviousLeg;
                                FlybyEvaluation refinedFlyby;
                                if (exactGeometry && TryRefineIncomingLegForFlyby(previousLeg, leg.DepartureVInfVector, out refinedPreviousLeg, out refinedFlyby))
                                {
                                    if (next.Legs.Count == 1 && previousLeg.From == request.Origin)
                                    {
                                        double oldEjection = GetRouteLegEjectionDV(previousLeg, request.ParkingAltitude);
                                        double newEjection = GetRouteLegEjectionDV(refinedPreviousLeg, request.ParkingAltitude);
                                        if (IsFinite(oldEjection) && IsFinite(newEjection))
                                        {
                                            double correction = newEjection - oldEjection;
                                            next.Cost += correction;
                                            next.EjectionDV += correction;
                                        }
                                    }

                                    if (IsFinite(refinedFlyby.PoweredDV))
                                    {
                                        double flybyCorrection = refinedFlyby.PoweredDV - appliedFlyby.PoweredDV;
                                        next.Cost += flybyCorrection;
                                        next.FlybyDV += flybyCorrection;
                                        appliedFlyby = refinedFlyby;
                                    }

                                    next.Legs[next.Legs.Count - 1] = refinedPreviousLeg;
                                    previousLeg = refinedPreviousLeg;
                                }

                                previousLeg.FlybyPoweredDV = appliedFlyby.PoweredDV;
                                previousLeg.RequiredTurnDeg = appliedFlyby.RequiredTurnDeg;
                                previousLeg.FreeTurnDeg = appliedFlyby.FreeTurnDeg;
                                previousLeg.FlybyAssistName = appliedFlyby.AssistName;
                                previousLeg.PlannedFlybyPeriapsisRadius = appliedFlyby.PeriapsisRadius;
                                next.Legs[next.Legs.Count - 1] = previousLeg;
                            }
                            if (appliedCapture != null)
                            {
                                leg.CaptureDV = appliedCapture.CaptureDV;
                                leg.CapturePoweredDV = appliedCapture.PoweredDV;
                                leg.CaptureAssistName = appliedCapture.AssistName;
                                leg.CapturePostAssistVInf = appliedCapture.PostAssistVInf;
                                leg.CaptureTurnDeg = appliedCapture.RequiredTurnDeg;
                                leg.CaptureFreeTurnDeg = appliedCapture.FreeTurnDeg;
                            }
                            next.Legs.Add(leg);

                            if (IsFinite(next.Cost))
                            {
                                nextBeam.Add(next);
                            }
                        }
                    }
                }

                if (nextBeam.Count == 0)
                {
                    return false;
                }

                nextBeam.Sort(delegate (RouteState a, RouteState b)
                {
                    return a.Cost.CompareTo(b.Cost);
                });

                if (nextBeam.Count > request.BeamWidth)
                {
                    nextBeam.RemoveRange(request.BeamWidth, nextBeam.Count - request.BeamWidth);
                }

                beam = nextBeam;
            }

            if (beam.Count == 0)
            {
                return false;
            }

            RouteState best = beam[0];
            RouteNode[] routeCopy = new RouteNode[route.Length];
            Array.Copy(route, routeCopy, route.Length);

            CelestialBody[] routeBodies = new CelestialBody[routeCopy.Length];
            for (int i = 0; i < routeCopy.Length; i++)
            {
                routeBodies[i] = routeCopy[i].Body;
            }

            CelestialBody finalBody = routeCopy[routeCopy.Length - 1].Body;
            double resultDepartureUT = best.Legs.Count > 0 ? best.Legs[0].DepartUT : departUT;
            result = new TransferResult
            {
                Origin = request.Origin,
                Destination = finalBody,
                DepartureUT = resultDepartureUT,
                ArrivalUT = best.Time,
                TimeOfFlight = best.Time - resultDepartureUT,
                TotalDV = best.Cost,
                EjectionDV = best.EjectionDV,
                InsertionDV = best.InsertionDV,
                FlybyDV = best.FlybyDV,
                DepartureVInf = best.Legs.Count > 0 ? best.Legs[0].DepartureVInf : 0.0,
                ArrivalVInf = best.Legs.Count > 0 ? best.Legs[best.Legs.Count - 1].ArrivalVInf : 0.0,
                GravityRoute = true,
                RouteBodies = routeBodies,
                RouteNodes = routeCopy,
                EncounterUTs = best.Times.ToArray(),
                RouteLegs = best.Legs.ToArray(),
                RouteName = BuildRouteName(routeCopy)
            };

            return IsFinite(result.TotalDV);
        }

        private List<RouteLegResult> EvaluateLambertLeg(CelestialBody from, CelestialBody to, double departUT, double tof, double parkingAltitude, double captureAltitude)
        {
            List<RouteLegResult> results = new List<RouteLegResult>();
            if (tof <= DaySeconds() * 0.1)
            {
                return results;
            }

            LegGeometry geometry;
            if (!TryGetLegGeometry(from, to, out geometry))
            {
                return results;
            }

            if (geometry.Kind == LegKind.ParentToChild)
            {
                return EvaluateParentToChildLeg(geometry, departUT, tof, parkingAltitude);
            }

            if (geometry.Kind == LegKind.ChildToParent)
            {
                return EvaluateChildToParentLeg(geometry, departUT, tof, captureAltitude);
            }

            CelestialBody central = geometry.CentralBody;
            double arrivalUT = departUT + tof;
            Vector3d r1 = GetBodyPositionRelativeTo(from, central, departUT);
            Vector3d r2 = GetBodyPositionRelativeTo(to, central, arrivalUT);
            Vector3d fromVel = GetBodyVelocityRelativeTo(from, central, departUT);
            Vector3d toVel = GetBodyVelocityRelativeTo(to, central, arrivalUT);

            for (int longWayValue = 0; longWayValue < 2; longWayValue++)
            {
                bool longWay = longWayValue == 1;
                Vector3d transferV1;
                Vector3d transferV2;

                if (!LambertSolver.TrySolve(r1, r2, tof, central.gravParameter, longWay, out transferV1, out transferV2))
                {
                    continue;
                }

                Vector3d departureVInfVector = transferV1 - fromVel;
                Vector3d arrivalVInfVector = transferV2 - toVel;
                double departureVInf = departureVInfVector.magnitude;
                double arrivalVInf = arrivalVInfVector.magnitude;
                if (!IsFinite(departureVInf) || !IsFinite(arrivalVInf))
                {
                    continue;
                }

                results.Add(new RouteLegResult
                {
                    From = from,
                    To = to,
                    CentralBody = central,
                    Kind = geometry.Kind,
                    DepartUT = departUT,
                    ArrivalUT = arrivalUT,
                    TimeOfFlight = tof,
                    StartPosition = r1,
                    StartVelocity = transferV1,
                    EndPosition = r2,
                    StartBodyVelocity = fromVel,
                    EndBodyVelocity = toVel,
                    DepartureVInfVector = departureVInfVector,
                    ArrivalVInfVector = arrivalVInfVector,
                    DepartureVInf = departureVInf,
                    ArrivalVInf = arrivalVInf,
                    LongWay = longWay
                });
            }

            return results;
        }

        private List<RouteLegResult> EvaluateParentToChildLeg(LegGeometry geometry, double departUT, double tof, double parkingAltitude)
        {
            List<RouteLegResult> results = new List<RouteLegResult>();
            CelestialBody from = geometry.From;
            CelestialBody to = geometry.To;
            CelestialBody central = geometry.CentralBody;
            double arrivalUT = departUT + tof;
            double parkingRadius = from.Radius + Math.Max(0.0, parkingAltitude);
            if (!IsFinite(parkingRadius) || parkingRadius <= from.Radius || from.gravParameter <= 0.0)
            {
                return results;
            }

            Vector3d targetPosition = GetBodyPositionRelativeTo(to, central, arrivalUT);
            Vector3d targetVelocity = GetBodyVelocityRelativeTo(to, central, arrivalUT);
            Vector3d xAxis;
            Vector3d yAxis;
            Vector3d normal;
            if (!TryGetTransferPlaneBasis(to, central, departUT, out xAxis, out yAxis, out normal))
            {
                return results;
            }

            double circularSpeed = Math.Sqrt(from.gravParameter / parkingRadius);
            int phaseSamples = GetLocalOrbitPhaseSamples();
            for (int phaseIndex = 0; phaseIndex < phaseSamples; phaseIndex++)
            {
                double phase = 2.0 * Math.PI * phaseIndex / phaseSamples;
                Vector3d radialDirection = xAxis * Math.Cos(phase) + yAxis * Math.Sin(phase);
                if (radialDirection.sqrMagnitude < 1e-10)
                {
                    continue;
                }

                radialDirection = radialDirection.normalized;
                Vector3d startPosition = radialDirection * parkingRadius;
                Vector3d parkingVelocity = Vector3d.Cross(normal, radialDirection).normalized * circularSpeed;
                for (int longWayValue = 0; longWayValue < 2; longWayValue++)
                {
                    bool longWay = longWayValue == 1;
                    Vector3d transferV1;
                    Vector3d transferV2;
                    if (!LambertSolver.TrySolve(startPosition, targetPosition, tof, central.gravParameter, longWay, out transferV1, out transferV2))
                    {
                        continue;
                    }

                    Vector3d departureBurn = transferV1 - parkingVelocity;
                    Vector3d arrivalVInfVector = transferV2 - targetVelocity;
                    double departureDV = departureBurn.magnitude;
                    double arrivalVInf = arrivalVInfVector.magnitude;
                    if (!IsFinite(departureDV) || !IsFinite(arrivalVInf) || departureDV <= 0.0 || departureDV > 25000.0)
                    {
                        continue;
                    }

                    results.Add(new RouteLegResult
                    {
                        From = from,
                        To = to,
                        CentralBody = central,
                        Kind = geometry.Kind,
                        DepartUT = departUT,
                        ArrivalUT = arrivalUT,
                        TimeOfFlight = tof,
                        StartPosition = startPosition,
                        StartVelocity = transferV1,
                        EndPosition = targetPosition,
                        StartBodyVelocity = parkingVelocity,
                        EndBodyVelocity = targetVelocity,
                        DepartureVInfVector = transferV1,
                        ArrivalVInfVector = arrivalVInfVector,
                        DepartureVInf = transferV1.magnitude,
                        ArrivalVInf = arrivalVInf,
                        HasDepartureDVOverride = true,
                        DepartureDVOverride = departureDV,
                        LongWay = longWay
                    });
                }
            }

            return results;
        }

        private List<RouteLegResult> EvaluateChildToParentLeg(LegGeometry geometry, double departUT, double tof, double captureAltitude)
        {
            List<RouteLegResult> results = new List<RouteLegResult>();
            CelestialBody from = geometry.From;
            CelestialBody to = geometry.To;
            CelestialBody central = geometry.CentralBody;
            double arrivalUT = departUT + tof;
            double captureRadius = to.Radius + Math.Max(0.0, captureAltitude);
            if (!IsFinite(captureRadius) || captureRadius <= to.Radius || to.gravParameter <= 0.0)
            {
                return results;
            }

            Vector3d startPosition = GetBodyPositionRelativeTo(from, central, departUT);
            Vector3d startVelocity = GetBodyVelocityRelativeTo(from, central, departUT);
            Vector3d xAxis;
            Vector3d yAxis;
            Vector3d normal;
            if (!TryGetTransferPlaneBasis(from, central, arrivalUT, out xAxis, out yAxis, out normal))
            {
                return results;
            }

            double circularSpeed = Math.Sqrt(to.gravParameter / captureRadius);
            int phaseSamples = GetLocalOrbitPhaseSamples();
            for (int phaseIndex = 0; phaseIndex < phaseSamples; phaseIndex++)
            {
                double phase = 2.0 * Math.PI * phaseIndex / phaseSamples;
                Vector3d radialDirection = xAxis * Math.Cos(phase) + yAxis * Math.Sin(phase);
                if (radialDirection.sqrMagnitude < 1e-10)
                {
                    continue;
                }

                radialDirection = radialDirection.normalized;
                Vector3d targetPosition = radialDirection * captureRadius;
                Vector3d targetOrbitVelocity = Vector3d.Cross(normal, radialDirection).normalized * circularSpeed;
                for (int longWayValue = 0; longWayValue < 2; longWayValue++)
                {
                    bool longWay = longWayValue == 1;
                    Vector3d transferV1;
                    Vector3d transferV2;
                    if (!LambertSolver.TrySolve(startPosition, targetPosition, tof, central.gravParameter, longWay, out transferV1, out transferV2))
                    {
                        continue;
                    }

                    Vector3d departureVInfVector = transferV1 - startVelocity;
                    Vector3d insertionBurn = targetOrbitVelocity - transferV2;
                    double departureVInf = departureVInfVector.magnitude;
                    double insertionDV = insertionBurn.magnitude;
                    if (!IsFinite(departureVInf) || !IsFinite(insertionDV) || insertionDV < 0.0 || insertionDV > 25000.0)
                    {
                        continue;
                    }

                    results.Add(new RouteLegResult
                    {
                        From = from,
                        To = to,
                        CentralBody = central,
                        Kind = geometry.Kind,
                        DepartUT = departUT,
                        ArrivalUT = arrivalUT,
                        TimeOfFlight = tof,
                        StartPosition = startPosition,
                        StartVelocity = transferV1,
                        EndPosition = targetPosition,
                        StartBodyVelocity = startVelocity,
                        EndBodyVelocity = targetOrbitVelocity,
                        DepartureVInfVector = departureVInfVector,
                        ArrivalVInfVector = transferV2,
                        DepartureVInf = departureVInf,
                        ArrivalVInf = transferV2.magnitude,
                        HasInsertionDVOverride = true,
                        InsertionDVOverride = insertionDV,
                        PlannedCapturePeriapsisRadius = captureRadius,
                        LongWay = longWay
                    });
                }
            }

            return results;
        }

        private List<RouteLegResult> EvaluateActiveVesselFirstLeg(Orbit vesselOrbit, CelestialBody from, CelestialBody to, double nominalDepartUT, double tof, double minDepartUT, double parkingAltitude, double captureAltitude, int phaseSamples)
        {
            List<RouteLegResult> results = new List<RouteLegResult>();
            if (vesselOrbit == null || from == null || to == null || from.orbit == null || from.orbit.referenceBody == null || tof <= DaySeconds() * 0.1)
            {
                return results;
            }

            LegGeometry geometry;
            if (!TryGetLegGeometry(from, to, out geometry))
            {
                return results;
            }

            CelestialBody central = geometry.CentralBody;
            double period = vesselOrbit.period;
            if (!IsFinite(period) || period <= 1.0 || period > DaySeconds() * 20.0)
            {
                period = DaySeconds();
            }

            double now = Planetarium.GetUniversalTime() + 60.0;
            double earliest = Math.Max(now, minDepartUT);
            phaseSamples = Math.Max(3, phaseSamples);
            double phaseStart = Math.Max(earliest, nominalDepartUT - period * 0.5);
            for (int phaseIndex = 0; phaseIndex < phaseSamples; phaseIndex++)
            {
                double departUT = phaseStart + period * phaseIndex / Math.Max(1, phaseSamples - 1);
                if (geometry.Kind == LegKind.ParentToChild && vesselOrbit.referenceBody == from)
                {
                    double arrivalUT = departUT + tof;
                    Vector3d originPosition = GetBodyPositionRelativeTo(from, central, departUT);
                    Vector3d originVelocity = GetBodyVelocityRelativeTo(from, central, departUT);
                    Vector3d vesselPosition = vesselOrbit.getRelativePositionAtUT(departUT);
                    Vector3d vesselVelocity = vesselOrbit.getOrbitalVelocityAtUT(departUT);
                    Vector3d startPosition = originPosition + vesselPosition;
                    Vector3d startVelocity = originVelocity + vesselVelocity;
                    Vector3d targetPosition = GetBodyPositionRelativeTo(to, central, arrivalUT);
                    Vector3d targetVelocity = GetBodyVelocityRelativeTo(to, central, arrivalUT);

                    for (int longWayValue = 0; longWayValue < 2; longWayValue++)
                    {
                        bool longWay = longWayValue == 1;
                        Vector3d transferV1;
                        Vector3d transferV2;
                        if (!LambertSolver.TrySolve(startPosition, targetPosition, tof, central.gravParameter, longWay, out transferV1, out transferV2))
                        {
                            continue;
                        }

                        Vector3d worldDelta = transferV1 - startVelocity;
                        Vector3d maneuverDelta = WorldDeltaToManeuverDelta(vesselOrbit, departUT, worldDelta);
                        Vector3d arrivalVInfVector = transferV2 - targetVelocity;
                        double ejectionDV = maneuverDelta.magnitude;
                        double arrivalVInf = arrivalVInfVector.magnitude;
                        if (!IsFinite(ejectionDV) || !IsFinite(arrivalVInf) || ejectionDV <= 0.0 || ejectionDV > 25000.0)
                        {
                            continue;
                        }

                        results.Add(new RouteLegResult
                        {
                            From = from,
                            To = to,
                            CentralBody = central,
                            Kind = geometry.Kind,
                            DepartUT = departUT,
                            ArrivalUT = arrivalUT,
                            TimeOfFlight = tof,
                            StartPosition = startPosition,
                            StartVelocity = transferV1,
                            EndPosition = targetPosition,
                            StartBodyVelocity = startVelocity,
                            EndBodyVelocity = targetVelocity,
                            DepartureVInfVector = transferV1 - originVelocity,
                            ArrivalVInfVector = arrivalVInfVector,
                            DepartureVInf = (transferV1 - originVelocity).magnitude,
                            ArrivalVInf = arrivalVInf,
                            StartsFromActiveVessel = true,
                            ActiveVesselOrbit = vesselOrbit,
                            ActiveVesselWorldDeltaV = worldDelta,
                            ActiveVesselDepartureDV = ejectionDV,
                            HasDepartureDVOverride = true,
                            DepartureDVOverride = ejectionDV,
                            LongWay = longWay
                        });
                    }

                    continue;
                }

                List<RouteLegResult> legOptions = EvaluateLambertLeg(from, to, departUT, tof, parkingAltitude, captureAltitude);
                for (int optionIndex = 0; optionIndex < legOptions.Count; optionIndex++)
                {
                    RouteLegResult leg = legOptions[optionIndex];
                    Vector3d maneuverDelta;
                    if (!TryComputeEjectionBurnFromVInf(vesselOrbit, from, leg.DepartUT, leg.DepartureVInfVector, out maneuverDelta))
                    {
                        continue;
                    }

                    Vector3d worldDelta = ManeuverDeltaToWorldDelta(vesselOrbit, leg.DepartUT, maneuverDelta);
                    double ejectionDV = maneuverDelta.magnitude;
                    if (!IsFinite(ejectionDV) || ejectionDV <= 0.0 || ejectionDV > 25000.0)
                    {
                        continue;
                    }

                    leg.StartsFromActiveVessel = true;
                    leg.ActiveVesselOrbit = vesselOrbit;
                    leg.ActiveVesselWorldDeltaV = worldDelta;
                    leg.ActiveVesselDepartureDV = ejectionDV;
                    leg.HasDepartureDVOverride = true;
                    leg.DepartureDVOverride = ejectionDV;
                    results.Add(leg);
                }
            }

            return results;
        }

        private static double GetRouteLegEjectionDV(RouteLegResult leg, double parkingAltitude)
        {
            if (leg != null && leg.StartsFromActiveVessel)
            {
                return leg.ActiveVesselDepartureDV;
            }

            if (leg != null && leg.HasDepartureDVOverride)
            {
                return leg.DepartureDVOverride;
            }

            return leg != null ? CircularOrbitToHyperbolaDV(leg.From, parkingAltitude, leg.DepartureVInf) : double.PositiveInfinity;
        }

        private static double GetRouteLegInsertionDV(RouteLegResult leg, double captureAltitude)
        {
            if (leg != null && leg.HasInsertionDVOverride)
            {
                return leg.InsertionDVOverride;
            }

            return leg != null ? CircularOrbitToHyperbolaDV(leg.To, captureAltitude, leg.ArrivalVInf) : double.PositiveInfinity;
        }

        private static Vector3d GetRouteLegStartPosition(RouteLegResult leg)
        {
            return leg != null ? leg.StartPosition : Vector3d.zero;
        }

        private static Vector3d GetRouteLegPreBurnVelocity(RouteLegResult leg)
        {
            if (leg == null)
            {
                return Vector3d.zero;
            }

            if (IsFinite(leg.StartBodyVelocity.magnitude))
            {
                return leg.StartBodyVelocity;
            }

            return GetBodyVelocityRelativeTo(leg.From, leg.CentralBody, leg.DepartUT);
        }

        private static bool TryUpdateActiveVesselDeparture(RouteLegResult leg, Vector3d desiredVInf)
        {
            if (leg == null || !leg.StartsFromActiveVessel)
            {
                return true;
            }
            if (leg.ActiveVesselOrbit == null || leg.From == null || desiredVInf.sqrMagnitude < 1e-8)
            {
                return false;
            }

            if (leg.Kind == LegKind.ParentToChild && leg.ActiveVesselOrbit.referenceBody == leg.From)
            {
                Vector3d worldDelta = leg.StartVelocity - leg.StartBodyVelocity;
                Vector3d directManeuverDelta = WorldDeltaToManeuverDelta(leg.ActiveVesselOrbit, leg.DepartUT, worldDelta);
                leg.ActiveVesselWorldDeltaV = worldDelta;
                leg.ActiveVesselDepartureDV = directManeuverDelta.magnitude;
                leg.HasDepartureDVOverride = true;
                leg.DepartureDVOverride = leg.ActiveVesselDepartureDV;
                return IsFinite(leg.ActiveVesselDepartureDV) && leg.ActiveVesselDepartureDV > 0.0;
            }

            Vector3d maneuverDelta;
            if (!TryComputeEjectionBurnFromVInf(leg.ActiveVesselOrbit, leg.From, leg.DepartUT, desiredVInf, out maneuverDelta))
            {
                return false;
            }

            leg.ActiveVesselWorldDeltaV = ManeuverDeltaToWorldDelta(leg.ActiveVesselOrbit, leg.DepartUT, maneuverDelta);
            leg.ActiveVesselDepartureDV = maneuverDelta.magnitude;
            return IsFinite(leg.ActiveVesselDepartureDV) && leg.ActiveVesselDepartureDV > 0.0;
        }

        private bool TryRefineIncomingLegForFlyby(RouteLegResult incomingLeg, Vector3d desiredOutgoingVInf, out RouteLegResult refinedLeg, out FlybyEvaluation flyby)
        {
            refinedLeg = null;
            flyby = null;
            if (incomingLeg == null || incomingLeg.From == null || incomingLeg.To == null || incomingLeg.CentralBody == null)
            {
                return false;
            }

            Vector3d incomingVInf = incomingLeg.ArrivalVInfVector;
            RouteLegResult current = incomingLeg.Clone();
            for (int iteration = 0; iteration < 5; iteration++)
            {
                flyby = EvaluateFlyby(current.To, incomingVInf, desiredOutgoingVInf);
                if (flyby == null || !flyby.IsValid)
                {
                    return false;
                }

                Vector3d targetOffset;
                if (!TryComputeFlybyEntryOffset(current.To, incomingVInf, desiredOutgoingVInf, flyby.PeriapsisRadius, out targetOffset))
                {
                    return false;
                }

                Vector3d startPosition = GetRouteLegStartPosition(current);
                Vector3d targetPosition = GetBodyPositionRelativeTo(current.To, current.CentralBody, current.ArrivalUT) + targetOffset;
                Vector3d transferVelocity;
                Vector3d arrivalVelocity;
                if (!LambertSolver.TrySolve(startPosition, targetPosition, current.TimeOfFlight, current.CentralBody.gravParameter, current.LongWay, out transferVelocity, out arrivalVelocity))
                {
                    return false;
                }

                Vector3d fromVelocity = GetRouteLegPreBurnVelocity(current);
                Vector3d toVelocity = GetBodyVelocityRelativeTo(current.To, current.CentralBody, current.ArrivalUT);
                Vector3d refinedIncomingVInf = arrivalVelocity - toVelocity;
                Vector3d refinedDepartureVInf = transferVelocity - GetBodyVelocityRelativeTo(current.From, current.CentralBody, current.DepartUT);
                if (!IsFinite(refinedIncomingVInf.magnitude) || !IsFinite(refinedDepartureVInf.magnitude))
                {
                    return false;
                }

                current.StartPosition = startPosition;
                current.StartVelocity = transferVelocity;
                current.EndPosition = targetPosition;
                current.DepartureVInfVector = refinedDepartureVInf;
                current.ArrivalVInfVector = refinedIncomingVInf;
                current.DepartureVInf = refinedDepartureVInf.magnitude;
                current.ArrivalVInf = refinedIncomingVInf.magnitude;
                if (current.HasDepartureDVOverride)
                {
                    current.DepartureDVOverride = (transferVelocity - current.StartBodyVelocity).magnitude;
                }
                if (!TryUpdateActiveVesselDeparture(current, refinedDepartureVInf))
                {
                    return false;
                }
                current.HasTargetOffset = true;
                current.TargetOffset = targetOffset;
                current.PlannedFlybyPeriapsisRadius = flyby.PeriapsisRadius;

                if (RelativeVectorError(refinedIncomingVInf, incomingVInf) < 0.002)
                {
                    break;
                }

                incomingVInf = refinedIncomingVInf;
            }

            flyby = EvaluateFlyby(current.To, current.ArrivalVInfVector, desiredOutgoingVInf);
            if (flyby == null || !flyby.IsValid)
            {
                return false;
            }

            Vector3d finalTargetOffset;
            if (TryComputeFlybyEntryOffset(current.To, current.ArrivalVInfVector, desiredOutgoingVInf, flyby.PeriapsisRadius, out finalTargetOffset))
            {
                Vector3d startPosition = GetRouteLegStartPosition(current);
                Vector3d targetPosition = GetBodyPositionRelativeTo(current.To, current.CentralBody, current.ArrivalUT) + finalTargetOffset;
                Vector3d transferVelocity;
                Vector3d arrivalVelocity;
                if (LambertSolver.TrySolve(startPosition, targetPosition, current.TimeOfFlight, current.CentralBody.gravParameter, current.LongWay, out transferVelocity, out arrivalVelocity))
                {
                    Vector3d fromVelocity = GetRouteLegPreBurnVelocity(current);
                    Vector3d toVelocity = GetBodyVelocityRelativeTo(current.To, current.CentralBody, current.ArrivalUT);
                    Vector3d refinedIncomingVInf = arrivalVelocity - toVelocity;
                    Vector3d refinedDepartureVInf = transferVelocity - GetBodyVelocityRelativeTo(current.From, current.CentralBody, current.DepartUT);
                    if (IsFinite(refinedIncomingVInf.magnitude) && IsFinite(refinedDepartureVInf.magnitude))
                    {
                        current.StartPosition = startPosition;
                        current.StartVelocity = transferVelocity;
                        current.DepartureVInfVector = refinedDepartureVInf;
                        current.ArrivalVInfVector = refinedIncomingVInf;
                        current.DepartureVInf = refinedDepartureVInf.magnitude;
                        current.ArrivalVInf = refinedIncomingVInf.magnitude;
                        if (current.HasDepartureDVOverride)
                        {
                            current.DepartureDVOverride = (transferVelocity - current.StartBodyVelocity).magnitude;
                        }
                        TryUpdateActiveVesselDeparture(current, refinedDepartureVInf);
                    }
                }

                current.HasTargetOffset = true;
                current.TargetOffset = finalTargetOffset;
                current.EndPosition = targetPosition;
                current.PlannedFlybyPeriapsisRadius = flyby.PeriapsisRadius;
            }

            FlybyEvaluation finalFlyby = EvaluateFlyby(current.To, current.ArrivalVInfVector, desiredOutgoingVInf);
            if (finalFlyby != null && finalFlyby.IsValid)
            {
                flyby = finalFlyby;
                current.PlannedFlybyPeriapsisRadius = finalFlyby.PeriapsisRadius;
            }

            refinedLeg = current;
            return true;
        }

        private bool TryApplyCaptureToLeg(RouteLegResult routeLeg, double captureAltitude, bool exactGeometry, bool includeMoons, out RouteLegResult capturedLeg, out CaptureEvaluation capture)
        {
            capturedLeg = routeLeg;
            capture = null;
            if (routeLeg == null || routeLeg.To == null)
            {
                return false;
            }

            if (routeLeg.HasInsertionDVOverride)
            {
                double insertionDV = GetRouteLegInsertionDV(routeLeg, captureAltitude);
                if (!IsFinite(insertionDV) || insertionDV < 0.0)
                {
                    return false;
                }

                capture = new CaptureEvaluation
                {
                    IsValid = true,
                    CaptureDV = insertionDV,
                    PoweredDV = 0.0,
                    TotalDV = insertionDV,
                    PostAssistVInf = 0.0,
                    RequiredTurnDeg = 0.0,
                    FreeTurnDeg = 0.0,
                    AssistName = routeLeg.To.bodyName,
                    AssistUT = routeLeg.ArrivalUT
                };

                capturedLeg.CaptureDV = capture.CaptureDV;
                capturedLeg.CapturePoweredDV = capture.PoweredDV;
                capturedLeg.CaptureAssistName = capture.AssistName;
                capturedLeg.CapturePostAssistVInf = capture.PostAssistVInf;
                capturedLeg.CaptureTurnDeg = capture.RequiredTurnDeg;
                capturedLeg.CaptureFreeTurnDeg = capture.FreeTurnDeg;
                if (!IsFinite(capturedLeg.PlannedCapturePeriapsisRadius) || capturedLeg.PlannedCapturePeriapsisRadius <= 0.0)
                {
                    capturedLeg.PlannedCapturePeriapsisRadius = GetCapturePeriapsisRadius(routeLeg.To, captureAltitude);
                }
                return true;
            }

            if (exactGeometry)
            {
                RouteLegResult refinedLeg;
                if (!TryRefineIncomingLegForCapture(routeLeg, captureAltitude, includeMoons, out refinedLeg, out capture))
                {
                    return false;
                }

                capturedLeg = refinedLeg;
            }
            else
            {
                capture = EvaluateBestCapture(routeLeg.To, routeLeg.ArrivalVInfVector, captureAltitude, routeLeg.ArrivalUT, includeMoons);
                if (capture == null || !capture.IsValid)
                {
                    return false;
                }

                capturedLeg.PlannedCapturePeriapsisRadius = GetCapturePeriapsisRadius(routeLeg.To, captureAltitude);
            }

            capturedLeg.CaptureDV = capture.CaptureDV;
            capturedLeg.CapturePoweredDV = capture.PoweredDV;
            capturedLeg.CaptureAssistName = capture.AssistName;
            capturedLeg.CapturePostAssistVInf = capture.PostAssistVInf;
            capturedLeg.CaptureTurnDeg = capture.RequiredTurnDeg;
            capturedLeg.CaptureFreeTurnDeg = capture.FreeTurnDeg;
            return true;
        }

        private bool TryRefineIncomingLegForCapture(RouteLegResult incomingLeg, double captureAltitude, bool includeMoons, out RouteLegResult refinedLeg, out CaptureEvaluation capture)
        {
            refinedLeg = null;
            capture = null;
            if (incomingLeg == null || incomingLeg.From == null || incomingLeg.To == null || incomingLeg.CentralBody == null)
            {
                return false;
            }

            double periapsisRadius = GetCapturePeriapsisRadius(incomingLeg.To, captureAltitude);
            if (!IsFinite(periapsisRadius) || periapsisRadius <= incomingLeg.To.Radius)
            {
                return false;
            }

            Vector3d targetPosition = GetBodyPositionRelativeTo(incomingLeg.To, incomingLeg.CentralBody, incomingLeg.ArrivalUT);
            Vector3d targetVelocity = GetBodyVelocityRelativeTo(incomingLeg.To, incomingLeg.CentralBody, incomingLeg.ArrivalUT);
            Vector3d capturePlaneNormal = SafeNormal(targetPosition, targetVelocity);

            RouteLegResult bestLeg = null;
            CaptureEvaluation bestCapture = null;
            double bestScore = double.PositiveInfinity;
            double adjustedCaptureAltitude = Math.Max(0.0, periapsisRadius - incomingLeg.To.Radius);

            for (int sideIndex = 0; sideIndex < 2; sideIndex++)
            {
                double side = sideIndex == 0 ? 1.0 : -1.0;
                RouteLegResult current = incomingLeg.Clone();
                Vector3d incomingVInf = current.ArrivalVInfVector;
                bool success = true;

                for (int iteration = 0; iteration < 5; iteration++)
                {
                    Vector3d targetOffset;
                    if (!TryComputeCaptureEntryOffset(current.To, incomingVInf, periapsisRadius, capturePlaneNormal, side, out targetOffset))
                    {
                        success = false;
                        break;
                    }

                    Vector3d startPosition = GetRouteLegStartPosition(current);
                    Vector3d refinedTargetPosition = GetBodyPositionRelativeTo(current.To, current.CentralBody, current.ArrivalUT) + targetOffset;
                    Vector3d transferVelocity;
                    Vector3d arrivalVelocity;
                    if (!LambertSolver.TrySolve(startPosition, refinedTargetPosition, current.TimeOfFlight, current.CentralBody.gravParameter, current.LongWay, out transferVelocity, out arrivalVelocity))
                    {
                        success = false;
                        break;
                    }

                    Vector3d fromVelocity = GetRouteLegPreBurnVelocity(current);
                    Vector3d toVelocity = GetBodyVelocityRelativeTo(current.To, current.CentralBody, current.ArrivalUT);
                    Vector3d refinedIncomingVInf = arrivalVelocity - toVelocity;
                    Vector3d refinedDepartureVInf = transferVelocity - GetBodyVelocityRelativeTo(current.From, current.CentralBody, current.DepartUT);
                    if (!IsFinite(refinedIncomingVInf.magnitude) || !IsFinite(refinedDepartureVInf.magnitude))
                    {
                        success = false;
                        break;
                    }

                    current.StartPosition = startPosition;
                    current.StartVelocity = transferVelocity;
                    current.EndPosition = refinedTargetPosition;
                    current.DepartureVInfVector = refinedDepartureVInf;
                    current.ArrivalVInfVector = refinedIncomingVInf;
                    current.DepartureVInf = refinedDepartureVInf.magnitude;
                    current.ArrivalVInf = refinedIncomingVInf.magnitude;
                    if (current.HasDepartureDVOverride)
                    {
                        current.DepartureDVOverride = (transferVelocity - current.StartBodyVelocity).magnitude;
                    }
                    if (!TryUpdateActiveVesselDeparture(current, refinedDepartureVInf))
                    {
                        success = false;
                        break;
                    }
                    current.HasTargetOffset = true;
                    current.TargetOffset = targetOffset;
                    current.PlannedCapturePeriapsisRadius = periapsisRadius;

                    if (RelativeVectorError(refinedIncomingVInf, incomingVInf) < 0.002)
                    {
                        break;
                    }

                    incomingVInf = refinedIncomingVInf;
                }

                if (!success)
                {
                    continue;
                }

                CaptureEvaluation candidateCapture = EvaluateBestCapture(current.To, current.ArrivalVInfVector, adjustedCaptureAltitude, current.ArrivalUT, includeMoons);
                if (candidateCapture == null || !candidateCapture.IsValid)
                {
                    continue;
                }

                double departureScore = current.StartsFromActiveVessel ? current.ActiveVesselDepartureDV : current.DepartureVInf;
                double score = candidateCapture.TotalDV + Math.Max(0.0, departureScore) * 0.05;
                if (IsFinite(score) && score < bestScore)
                {
                    bestScore = score;
                    bestLeg = current;
                    bestCapture = candidateCapture;
                }
            }

            if (bestLeg == null || bestCapture == null)
            {
                return false;
            }

            refinedLeg = bestLeg;
            capture = bestCapture;
            return true;
        }

        private static bool TryComputeFlybyEntryOffset(CelestialBody body, Vector3d incomingVInf, Vector3d outgoingVInf, double periapsisRadius, out Vector3d offset)
        {
            offset = Vector3d.zero;
            if (body == null || incomingVInf.sqrMagnitude < 1e-8 || outgoingVInf.sqrMagnitude < 1e-8 || !IsFinite(periapsisRadius))
            {
                return false;
            }

            double soi = GetSafeSphereOfInfluence(body);
            double vin = incomingVInf.magnitude;
            double mu = body.gravParameter;
            double rp = Clamp(periapsisRadius, body.Radius + 1.0, Math.Max(body.Radius + 1.0, soi * 0.95));
            double b = rp * Math.Sqrt(1.0 + 2.0 * mu / Math.Max(1.0, rp * vin * vin));
            if (!IsFinite(b) || b <= 0.0)
            {
                return false;
            }

            b = Math.Min(b, soi * 0.92);
            Vector3d incomingDirection = incomingVInf.normalized;
            Vector3d turnDirection = outgoingVInf - incomingDirection * Vector3d.Dot(outgoingVInf, incomingDirection);
            if (turnDirection.sqrMagnitude < 1e-10)
            {
                Vector3d normal = Vector3d.Cross(incomingDirection, new Vector3d(0.0, 1.0, 0.0));
                if (normal.sqrMagnitude < 1e-10)
                {
                    normal = Vector3d.Cross(incomingDirection, new Vector3d(1.0, 0.0, 0.0));
                }
                turnDirection = Vector3d.Cross(normal.normalized, incomingDirection);
            }

            Vector3d bVector = -turnDirection.normalized * b;
            double along = Math.Sqrt(Math.Max(0.0, soi * soi - b * b));
            offset = -incomingDirection * along + bVector;
            return IsFinite(offset.magnitude) && offset.sqrMagnitude > 1.0;
        }

        private static bool TryComputeCaptureEntryOffset(CelestialBody body, Vector3d incomingVInf, double periapsisRadius, Vector3d planeNormal, double side, out Vector3d offset)
        {
            offset = Vector3d.zero;
            if (body == null || incomingVInf.sqrMagnitude < 1e-8 || !IsFinite(periapsisRadius))
            {
                return false;
            }

            double soi = GetSafeSphereOfInfluence(body);
            double vin = incomingVInf.magnitude;
            double mu = body.gravParameter;
            double rp = Clamp(periapsisRadius, body.Radius + 1.0, Math.Max(body.Radius + 1.0, soi * 0.95));
            double b = rp * Math.Sqrt(1.0 + 2.0 * mu / Math.Max(1.0, rp * vin * vin));
            if (!IsFinite(b) || b <= 0.0)
            {
                return false;
            }

            b = Math.Min(b, soi * 0.92);
            Vector3d incomingDirection = incomingVInf.normalized;
            Vector3d bDirection = Vector3d.Cross(planeNormal, incomingDirection);
            if (bDirection.sqrMagnitude < 1e-10)
            {
                Vector3d reference = Math.Abs(Vector3d.Dot(incomingDirection, new Vector3d(0.0, 1.0, 0.0))) < 0.85
                    ? new Vector3d(0.0, 1.0, 0.0)
                    : new Vector3d(1.0, 0.0, 0.0);
                bDirection = reference - incomingDirection * Vector3d.Dot(reference, incomingDirection);
            }
            if (bDirection.sqrMagnitude < 1e-10)
            {
                return false;
            }

            Vector3d bVector = bDirection.normalized * b * (side >= 0.0 ? 1.0 : -1.0);
            double along = Math.Sqrt(Math.Max(0.0, soi * soi - b * b));
            offset = -incomingDirection * along + bVector;
            return IsFinite(offset.magnitude) && offset.sqrMagnitude > 1.0;
        }

        private static double GetSeedLegDuration(TransferResult durationSeed, int legIndex)
        {
            if (durationSeed == null || durationSeed.RouteLegs == null || legIndex < 0 || legIndex >= durationSeed.RouteLegs.Length ||
                durationSeed.RouteLegs[legIndex] == null)
            {
                return double.NaN;
            }

            return durationSeed.RouteLegs[legIndex].TimeOfFlight;
        }

        private List<double> GenerateLegDurations(CelestialBody from, CelestialBody to, double currentUT, double maxArrivalUT, int sampleCount, double seedDuration = double.NaN)
        {
            List<double> durations = new List<double>(24);
            double maxDuration = Math.Max(0.0, maxArrivalUT - currentUT);
            if (maxDuration <= DaySeconds())
            {
                return durations;
            }

            double nominal = EstimateLegTime(from, to);
            double[] factors = sampleCount <= 5
                ? LegDurationFactorsFast
                : sampleCount <= 8
                    ? LegDurationFactorsNormal
                    : LegDurationFactorsAccurate;

            for (int i = 0; i < factors.Length; i++)
            {
                AddDurationIfValid(durations, nominal * factors[i], maxDuration);
            }

            if (IsFinite(seedDuration) && seedDuration > 0.0)
            {
                for (int i = 0; i < LegDurationSeedFactors.Length; i++)
                {
                    AddDurationIfValid(durations, seedDuration * LegDurationSeedFactors[i], maxDuration);
                }
            }

            int uniformSamples = Math.Max(3, Math.Min(8, sampleCount));
            for (int i = 1; i <= uniformSamples; i++)
            {
                AddDurationIfValid(durations, maxDuration * i / (uniformSamples + 1), maxDuration);
            }

            durations.Sort();
            return durations;
        }

        private static void AddDurationIfValid(List<double> durations, double duration, double maxDuration)
        {
            double minDuration = DaySeconds() * 2.0;
            if (!IsFinite(duration) || duration < minDuration || duration > maxDuration)
            {
                return;
            }

            for (int i = 0; i < durations.Count; i++)
            {
                if (Math.Abs(durations[i] - duration) < DaySeconds() * 0.5)
                {
                    return;
                }
            }

            durations.Add(duration);
        }

        private static double EstimateMinimumRemainingRouteTime(RouteNode[] route, int nextBodyIndex)
        {
            double total = 0.0;
            for (int i = nextBodyIndex; i < route.Length - 1; i++)
            {
                total += Math.Max(DaySeconds() * 2.0, EstimateLegTime(route[i].Body, route[i + 1].Body) * 0.28);
            }

            return total;
        }

        private static double EstimateLegTime(CelestialBody from, CelestialBody to)
        {
            if (from == to)
            {
                double period = from.orbit.period;
                if (IsFinite(period) && period > DaySeconds())
                {
                    return period * 0.92;
                }
                return YearSeconds();
            }

            LegGeometry geometry;
            if (!TryGetLegGeometry(from, to, out geometry))
            {
                return YearSeconds();
            }

            if ((geometry.Kind == LegKind.ParentToChild || geometry.Kind == LegKind.ChildToParent) && from.orbit != null)
            {
                Orbit moonOrbit = geometry.Kind == LegKind.ParentToChild ? to.orbit : from.orbit;
                if (moonOrbit != null && IsFinite(moonOrbit.period) && moonOrbit.period > DaySeconds())
                {
                    return Math.Max(DaySeconds() * 0.5, moonOrbit.period * 0.18);
                }
            }

            double now = Planetarium.GetUniversalTime();
            double mu = geometry.CentralBody.gravParameter;
            double r1 = EstimateOrbitRadiusAroundCentral(from, geometry.CentralBody, now);
            double r2 = EstimateOrbitRadiusAroundCentral(to, geometry.CentralBody, now);
            if (!IsFinite(mu) || mu <= 0.0 || !IsFinite(r1) || !IsFinite(r2) || r1 <= 0.0 || r2 <= 0.0)
            {
                return YearSeconds();
            }

            double a = 0.5 * (r1 + r2);
            double hohmann = Math.PI * Math.Sqrt(a * a * a / mu);
            return Math.Max(DaySeconds() * 5.0, hohmann);
        }

        private FlybyEvaluation EvaluateBestFlyby(CelestialBody body, Vector3d incomingVInf, Vector3d outgoingVInf, double encounterUT, bool includeMoons)
        {
            FlybyEvaluation best = EvaluateFlyby(body, incomingVInf, outgoingVInf);
            best.AssistName = body.bodyName;

            if (!includeMoons)
            {
                return best;
            }

            List<CelestialBody> moons = GetMoonsOf(body);
            for (int moonIndex = 0; moonIndex < moons.Count; moonIndex++)
            {
                CelestialBody moon = moons[moonIndex];
                for (int sample = -3; sample <= 3; sample++)
                {
                    double sampleUT = encounterUT + sample * Math.Max(DaySeconds() * 0.25, moon.orbit.period / 14.0);
                    Vector3d moonVel = moon.orbit.getOrbitalVelocityAtUT(sampleUT);
                    FlybyEvaluation moonFlyby = EvaluateFlyby(moon, incomingVInf - moonVel, outgoingVInf - moonVel);
                    if (!moonFlyby.IsValid)
                    {
                        continue;
                    }

                    moonFlyby.AssistName = moon.bodyName;
                    moonFlyby.AssistUT = sampleUT;
                    if (moonFlyby.PoweredDV < best.PoweredDV)
                    {
                        best = moonFlyby;
                    }
                }
            }

            return best;
        }

        private CaptureEvaluation EvaluateBestCapture(CelestialBody body, Vector3d arrivalVInfVector, double captureAltitude, double encounterUT, bool includeMoons)
        {
            double directCapture = CircularOrbitToHyperbolaDV(body, captureAltitude, arrivalVInfVector.magnitude);
            CaptureEvaluation best = new CaptureEvaluation
            {
                IsValid = IsFinite(directCapture),
                CaptureDV = directCapture,
                PoweredDV = 0.0,
                TotalDV = directCapture,
                PostAssistVInf = arrivalVInfVector.magnitude,
                AssistName = body.bodyName
            };

            if (!includeMoons || !best.IsValid)
            {
                return best;
            }

            List<CelestialBody> moons = GetMoonsOf(body);
            for (int moonIndex = 0; moonIndex < moons.Count; moonIndex++)
            {
                CelestialBody moon = moons[moonIndex];
                for (int sample = -3; sample <= 3; sample++)
                {
                    double sampleUT = encounterUT + sample * Math.Max(DaySeconds() * 0.25, moon.orbit.period / 14.0);
                    Vector3d moonVel = moon.orbit.getOrbitalVelocityAtUT(sampleUT);
                    Vector3d vinMoon = arrivalVInfVector - moonVel;
                    double vinMoonMag = vinMoon.magnitude;
                    if (!IsFinite(vinMoonMag) || vinMoonMag <= 1e-3 || moonVel.sqrMagnitude < 1e-9)
                    {
                        continue;
                    }

                    Vector3d idealOutMoon = -moonVel.normalized * vinMoonMag;
                    double requiredTurn = AngleRad(vinMoon, idealOutMoon);
                    double freeTurn = MaxFlybyTurnAngleRad(moon, vinMoonMag);
                    double actualTurn = Math.Min(requiredTurn, freeTurn);
                    Vector3d outMoon = RotateToward(vinMoon, idealOutMoon, actualTurn);
                    double powered = requiredTurn > freeTurn ? 2.0 * vinMoonMag * Math.Sin((requiredTurn - freeTurn) * 0.5) : 0.0;
                    Vector3d postPlanetVInf = moonVel + outMoon;
                    double postVInf = postPlanetVInf.magnitude;
                    double captureDV = CircularOrbitToHyperbolaDV(body, captureAltitude, postVInf);
                    double total = powered + captureDV;

                    if (IsFinite(total) && total < best.TotalDV)
                    {
                        best = new CaptureEvaluation
                        {
                            IsValid = true,
                            CaptureDV = captureDV,
                            PoweredDV = powered,
                            TotalDV = total,
                            PostAssistVInf = postVInf,
                            RequiredTurnDeg = requiredTurn * 180.0 / Math.PI,
                            FreeTurnDeg = freeTurn * 180.0 / Math.PI,
                            AssistName = moon.bodyName,
                            AssistUT = sampleUT
                        };
                    }
                }
            }

            return best;
        }

        private static FlybyEvaluation EvaluateFlyby(CelestialBody body, Vector3d incomingVInf, Vector3d outgoingVInf)
        {
            FlybyEvaluation result = new FlybyEvaluation();
            double vin = incomingVInf.magnitude;
            double vout = outgoingVInf.magnitude;
            if (!IsFinite(vin) || !IsFinite(vout) || vin <= 1e-3 || vout <= 1e-3)
            {
                result.IsValid = false;
                result.PoweredDV = double.PositiveInfinity;
                return result;
            }

            double turnRad = AngleRad(incomingVInf, outgoingVInf);
            double vinAvg = 0.5 * (vin + vout);
            double maxTurnRad = MaxFlybyTurnAngleRad(body, vinAvg);
            double magnitudeDifference = Math.Abs(vout - vin);
            double magnitudeTolerance = Math.Max(2.0, vinAvg * 0.0015);
            double turnToleranceRad = 0.03 * Math.PI / 180.0;
            double freeTurnRad = Math.Min(turnRad, maxTurnRad);
            double periapsisRadius = FlybyPeriapsisForTurn(body, vinAvg, freeTurnRad);
            double turnExcess = Math.Max(0.0, turnRad - maxTurnRad - turnToleranceRad);
            double speedExcess = Math.Max(0.0, magnitudeDifference - magnitudeTolerance);
            double poweredTurnDV = turnExcess > 0.0 ? 2.0 * vinAvg * Math.Sin(turnExcess * 0.5) : 0.0;
            double poweredDV = Math.Sqrt(speedExcess * speedExcess + poweredTurnDV * poweredTurnDV);
            if (!IsFinite(poweredDV) || poweredDV > 250000.0)
            {
                result.IsValid = false;
                result.PoweredDV = double.PositiveInfinity;
                result.RequiredTurnDeg = turnRad * 180.0 / Math.PI;
                result.FreeTurnDeg = maxTurnRad * 180.0 / Math.PI;
                result.PeriapsisRadius = periapsisRadius;
                return result;
            }

            result.IsValid = true;
            result.RequiredTurnDeg = turnRad * 180.0 / Math.PI;
            result.FreeTurnDeg = maxTurnRad * 180.0 / Math.PI;
            result.PoweredDV = poweredDV;
            result.PeriapsisRadius = periapsisRadius;
            return result;
        }

        private static double MaxFlybyTurnAngleRad(CelestialBody body, double vinf)
        {
            if (!IsFinite(vinf) || vinf <= 0.0 || body.gravParameter <= 0.0)
            {
                return 0.0;
            }

            double rp = SafeFlybyPeriapsisRadius(body);
            double eccentricity = 1.0 + rp * vinf * vinf / body.gravParameter;
            if (!IsFinite(eccentricity) || eccentricity <= 1.0)
            {
                return Math.PI;
            }

            return 2.0 * Math.Asin(Clamp(1.0 / eccentricity, 0.0, 1.0));
        }

        private static double FlybyPeriapsisForTurn(CelestialBody body, double vinf, double turnRad)
        {
            double safe = SafeFlybyPeriapsisRadius(body);
            if (!IsFinite(vinf) || vinf <= 0.0 || !IsFinite(turnRad) || turnRad <= 1e-6 || body.gravParameter <= 0.0)
            {
                return safe;
            }

            double sinHalf = Math.Sin(turnRad * 0.5);
            if (sinHalf <= 1e-6)
            {
                return GetSafeSphereOfInfluence(body) * 0.5;
            }

            double rp = body.gravParameter / (vinf * vinf) * (1.0 / sinHalf - 1.0);
            if (!IsFinite(rp))
            {
                return safe;
            }

            return Clamp(rp, safe, Math.Max(safe, GetSafeSphereOfInfluence(body) * 0.90));
        }

        private static double SafeFlybyPeriapsisRadius(CelestialBody body)
        {
            double altitude = Math.Max(GetMinimumSafeOrbitAltitude(body), Math.Max(10000.0, body.Radius * 0.02));
            if (body.atmosphere)
            {
                altitude = Math.Max(altitude, body.atmosphereDepth + 10000.0);
            }

            return Math.Max(body.Radius + altitude, body.Radius + 1.0);
        }

        private static double GetCapturePeriapsisRadius(CelestialBody body, double captureAltitude)
        {
            if (body == null)
            {
                return double.PositiveInfinity;
            }

            double safeAltitude = GetMinimumSafeOrbitAltitude(body);
            double requestedRadius = body.Radius + Math.Max(0.0, captureAltitude);
            double safeRadius = body.Radius + safeAltitude;
            double maxRadius = Math.Max(safeRadius, GetSafeSphereOfInfluence(body) * 0.90);
            return Clamp(Math.Max(requestedRadius, safeRadius), safeRadius, maxRadius);
        }

        private static double AngleRad(Vector3d a, Vector3d b)
        {
            if (a.sqrMagnitude < 1e-16 || b.sqrMagnitude < 1e-16)
            {
                return 0.0;
            }

            return Math.Acos(Clamp(Vector3d.Dot(a.normalized, b.normalized), -1.0, 1.0));
        }

        private List<CelestialBody> GetMoonsOf(CelestialBody body)
        {
            List<CelestialBody> moons = new List<CelestialBody>();
            for (int i = 0; i < bodies.Count; i++)
            {
                CelestialBody candidate = bodies[i];
                if (candidate != null && candidate.orbit != null && candidate.orbit.referenceBody == body)
                {
                    moons.Add(candidate);
                }
            }

            moons.Sort(delegate (CelestialBody a, CelestialBody b)
            {
                return Math.Abs(a.orbit.semiMajorAxis).CompareTo(Math.Abs(b.orbit.semiMajorAxis));
            });

            return moons;
        }

        private static Vector3d RotateToward(Vector3d from, Vector3d to, double angleRad)
        {
            if (from.sqrMagnitude < 1e-16 || to.sqrMagnitude < 1e-16 || angleRad <= 0.0)
            {
                return from;
            }

            double totalAngle = AngleRad(from, to);
            if (totalAngle <= 1e-8 || angleRad >= totalAngle)
            {
                return to.normalized * from.magnitude;
            }

            double t = angleRad / totalAngle;
            Vector3d a = from.normalized;
            Vector3d b = to.normalized;
            double sinTotal = Math.Sin(totalAngle);
            if (Math.Abs(sinTotal) < 1e-8)
            {
                return (a * (1.0 - t) + b * t).normalized * from.magnitude;
            }

            Vector3d rotated =
                a * (Math.Sin((1.0 - t) * totalAngle) / sinTotal) +
                b * (Math.Sin(t * totalAngle) / sinTotal);
            return rotated.normalized * from.magnitude;
        }

        private static int MaxRouteTemplateCount()
        {
            return 240;
        }

        private static int MaxTargetOrderCount()
        {
            return 24;
        }

        private static string BuildRouteName(RouteNode[] route)
        {
            if (route == null || route.Length == 0)
            {
                return "";
            }

            string text = route[0].Body.bodyName;
            for (int i = 1; i < route.Length; i++)
            {
                string suffix = "";
                if (route[i].IsMandatory)
                {
                    suffix = route[i].Mode == TargetVisitMode.Capture ? " (орбита)" : " (пролёт)";
                }

                text += " -> " + route[i].Body.bodyName + suffix;
            }

            return text;
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
                        if (TryEvaluateTransfer(request, depUT, tof, true, out result) && result.TotalDV < bestScore)
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

        private bool TryEvaluateTransfer(SearchRequest request, double departUT, double tof, bool exactGeometry, out TransferResult bestResultForPoint)
        {
            bestResultForPoint = null;

            if (tof <= 0.0)
            {
                return false;
            }

            CelestialBody origin = request.Origin;
            CelestialBody destination = request.Destination;
            List<RouteLegResult> legOptions = request.UseActiveVesselStart
                ? EvaluateActiveVesselFirstLeg(request.ActiveVesselOrbit, origin, destination, departUT, tof, request.StartUT, request.ParkingAltitude, request.CaptureAltitude, exactGeometry ? 9 : 5)
                : EvaluateLambertLeg(origin, destination, departUT, tof, request.ParkingAltitude, request.CaptureAltitude);

            TransferResult best = null;
            double bestScore = double.PositiveInfinity;
            for (int optionIndex = 0; optionIndex < legOptions.Count; optionIndex++)
            {
                RouteLegResult routeLeg = legOptions[optionIndex];
                CaptureEvaluation capture = null;
                if (request.IncludeCapture)
                {
                    RouteLegResult capturedLeg;
                    if (!TryApplyCaptureToLeg(routeLeg, request.CaptureAltitude, exactGeometry, false, out capturedLeg, out capture))
                    {
                        continue;
                    }

                    routeLeg = capturedLeg;
                }

                double ejection = GetRouteLegEjectionDV(routeLeg, request.ParkingAltitude);
                double insertion = request.IncludeCapture && capture != null ? capture.CaptureDV : 0.0;
                double captureTotal = request.IncludeCapture && capture != null ? capture.TotalDV : 0.0;
                double total = ejection + captureTotal;

                if (!IsFinite(total) || total <= 0.0)
                {
                    continue;
                }

                CelestialBody central = routeLeg.CentralBody;
                Vector3d currentVelocity = routeLeg.StartsFromActiveVessel
                    ? routeLeg.StartBodyVelocity
                    : GetRouteLegPreBurnVelocity(routeLeg);
                Vector3d departureVector = routeLeg.StartsFromActiveVessel && routeLeg.ActiveVesselWorldDeltaV.sqrMagnitude > 1e-8
                    ? routeLeg.ActiveVesselWorldDeltaV
                    : routeLeg.DepartureVInfVector;

                TransferResult candidate = new TransferResult
                {
                    Origin = origin,
                    Destination = destination,
                    DepartureUT = routeLeg.DepartUT,
                    ArrivalUT = routeLeg.ArrivalUT,
                    TimeOfFlight = routeLeg.TimeOfFlight,
                    TotalDV = total,
                    EjectionDV = ejection,
                    InsertionDV = insertion,
                    DepartureVInf = routeLeg.DepartureVInf,
                    ArrivalVInf = routeLeg.ArrivalVInf,
                    RouteBodies = new[] { origin, destination },
                    RouteNodes = new[]
                    {
                        new RouteNode { Body = origin, Mode = TargetVisitMode.Flyby, IsMandatory = true, TargetIndex = -1 },
                        new RouteNode { Body = destination, Mode = request.IncludeCapture ? TargetVisitMode.Capture : TargetVisitMode.Flyby, IsMandatory = true, TargetIndex = 0 }
                    },
                    EncounterUTs = new[] { routeLeg.DepartUT, routeLeg.ArrivalUT },
                    RouteLegs = new[] { routeLeg },
                    LongWay = routeLeg.LongWay,
                    PhaseAngleDeg = SignedAngleDeg(routeLeg.StartPosition, GetBodyPositionRelativeTo(destination, central, routeLeg.DepartUT), SafeNormal(routeLeg.StartPosition, currentVelocity)),
                    EjectionAngleDeg = SignedAngleDeg(currentVelocity, departureVector, SafeNormal(routeLeg.StartPosition, currentVelocity)),
                    EjectionInclinationDeg = InclinationToPlaneDeg(departureVector, SafeNormal(routeLeg.StartPosition, currentVelocity))
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

        private bool IsMapOverlayScene()
        {
            return HighLogic.LoadedScene == GameScenes.TRACKSTATION ||
                   HighLogic.LoadedScene == GameScenes.FLIGHT ||
                   (MapView.MapIsEnabled);
        }

        private void BuildTrajectoryOverlay(TransferResult result)
        {
            ClearTrajectoryOverlay();
            if (!showTrajectoryOverlay || result == null || result.RouteLegs == null || result.RouteLegs.Length == 0 || !IsMapOverlayScene() || ScaledSpace.Instance == null)
            {
                return;
            }

            trajectoryOverlayRoot = new GameObject("AutoTransferWindowPlanner_TrajectoryOverlay");
            AttachOverlayToScaledSpace(trajectoryOverlayRoot);
            overlayResult = result;

            for (int i = 0; i < result.RouteLegs.Length; i++)
            {
                AddTrajectoryLegOverlay(result.RouteLegs[i], i);
            }

            if (result.RouteBodies != null && result.EncounterUTs != null)
            {
                int count = Math.Min(result.RouteBodies.Length, result.EncounterUTs.Length);
                for (int i = 0; i < count; i++)
                {
                    AddGhostMarker(result.RouteBodies[i], result.EncounterUTs[i], i);
                }
            }
        }

        private void ClearTrajectoryOverlay()
        {
            if (trajectoryOverlayRoot != null)
            {
                Destroy(trajectoryOverlayRoot);
                trajectoryOverlayRoot = null;
            }

            overlayResult = null;
        }

        private void AddTrajectoryLegOverlay(RouteLegResult leg, int legIndex)
        {
            if (trajectoryOverlayRoot == null || leg.CentralBody == null)
            {
                return;
            }

            Orbit orbit = new Orbit();
            orbit.UpdateFromStateVectors(leg.StartPosition, leg.StartVelocity, leg.CentralBody, leg.DepartUT);

            int samples = 72;
            Vector3? previous = null;
            for (int i = 0; i < samples; i++)
            {
                double t = i / (double)(samples - 1);
                double ut = leg.DepartUT + leg.TimeOfFlight * t;
                Vector3d relativePosition = orbit.getRelativePositionAtUT(ut);
                Vector3d localPosition = GetBodyLocalPositionAtUT(leg.CentralBody, ut) + relativePosition;
                Vector3 scaledPosition = ToScaledVector3(localPosition);

                if (previous.HasValue && (i % 3) != 0)
                {
                    AddLineSegment(previous.Value, scaledPosition, legIndex, i);
                }

                previous = scaledPosition;
            }
        }

        private void AddLineSegment(Vector3 start, Vector3 end, int legIndex, int segmentIndex)
        {
            GameObject segment = new GameObject("ATWP_leg_" + legIndex + "_dash_" + segmentIndex);
            segment.transform.parent = trajectoryOverlayRoot.transform;
            segment.transform.localPosition = Vector3.zero;
            segment.layer = trajectoryOverlayRoot.layer;
            LineRenderer line = segment.AddComponent<LineRenderer>();
            line.useWorldSpace = false;
            line.material = GetTrajectoryLineMaterial();
            line.startColor = new Color(0.2f, 0.95f, 1.0f, 0.85f);
            line.endColor = new Color(1.0f, 0.85f, 0.15f, 0.85f);
            line.startWidth = 8f;
            line.endWidth = 8f;
            line.positionCount = 2;
            line.SetPosition(0, start);
            line.SetPosition(1, end);
        }

        private void AddGhostMarker(CelestialBody body, double ut, int index)
        {
            if (trajectoryOverlayRoot == null || body == null)
            {
                return;
            }

            GameObject marker = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            marker.name = "ATWP_ghost_" + body.bodyName + "_" + index;
            marker.transform.parent = trajectoryOverlayRoot.transform;
            marker.layer = trajectoryOverlayRoot.layer;
            marker.transform.localPosition = ToScaledVector3(GetBodyLocalPositionAtUT(body, ut));
            float scaledRadius = Mathf.Clamp((float)(body.Radius * ScaledSpace.ScaleFactor * 2.8), 14f, 70f);
            marker.transform.localScale = new Vector3(scaledRadius, scaledRadius, scaledRadius);

            Collider markerCollider = marker.GetComponent<Collider>();
            if (markerCollider != null)
            {
                Destroy(markerCollider);
            }

            Renderer renderer = marker.GetComponent<Renderer>();
            if (renderer != null)
            {
                renderer.material = GetTrajectoryMarkerMaterial();
            }
        }

        private static void AttachOverlayToScaledSpace(GameObject root)
        {
            if (root == null)
            {
                return;
            }

            ScaledSpace scaledSpace = ScaledSpace.Instance;
            if (scaledSpace != null)
            {
                root.transform.parent = scaledSpace.transform;
                root.transform.localPosition = Vector3.zero;
                root.transform.localRotation = Quaternion.identity;
                root.transform.localScale = Vector3.one;
                root.layer = scaledSpace.gameObject.layer;
            }
        }

        private Material GetTrajectoryLineMaterial()
        {
            if (trajectoryLineMaterial != null)
            {
                return trajectoryLineMaterial;
            }

            Material mapMaterial = MapView.DottedLineMaterialStatic;
            if (mapMaterial == null)
            {
                mapMaterial = MapView.OrbitLinesMaterialStatic;
            }

            if (mapMaterial != null)
            {
                trajectoryLineMaterial = new Material(mapMaterial);
            }
            else
            {
                Shader shader = Shader.Find("Particles/Additive");
                if (shader == null) shader = Shader.Find("Legacy Shaders/Particles/Additive");
                if (shader == null) shader = Shader.Find("Unlit/Color");
                trajectoryLineMaterial = new Material(shader);
            }

            trajectoryLineMaterial.color = new Color(0.2f, 0.95f, 1.0f, 0.85f);
            return trajectoryLineMaterial;
        }

        private Material GetTrajectoryMarkerMaterial()
        {
            if (trajectoryMarkerMaterial != null)
            {
                return trajectoryMarkerMaterial;
            }

            Shader shader = Shader.Find("Particles/Additive");
            if (shader == null) shader = Shader.Find("Legacy Shaders/Particles/Additive");
            if (shader == null) shader = Shader.Find("Unlit/Color");
            trajectoryMarkerMaterial = new Material(shader);
            trajectoryMarkerMaterial.color = new Color(1.0f, 0.95f, 0.2f, 0.75f);
            return trajectoryMarkerMaterial;
        }

        private static Vector3 ToScaledVector3(Vector3d localPosition)
        {
            Vector3d scaled = ScaledSpace.LocalToScaledSpace(localPosition);
            return new Vector3((float)scaled.x, (float)scaled.y, (float)scaled.z);
        }

        private static Vector3d GetBodyLocalPositionAtUT(CelestialBody body, double ut)
        {
            if (body == null || body.orbit == null)
            {
                return Vector3d.zero;
            }

            Vector3d position = body.orbit.getRelativePositionAtUT(ut);
            CelestialBody parent = body.orbit.referenceBody;
            if (parent != null && parent.orbit != null)
            {
                position += GetBodyLocalPositionAtUT(parent, ut);
            }

            return position;
        }

        private void CreateFirstManeuverNode()
        {
            if (bestResult == null || bestResult.RouteLegs == null || bestResult.RouteLegs.Length == 0)
            {
                SetStatus("Сначала рассчитай маршрут.", true);
                return;
            }

            if (HighLogic.LoadedScene != GameScenes.FLIGHT)
            {
                SetStatus("Манёвр можно создать только на активном аппарате в полёте или Map View.", true);
                return;
            }

            Vessel vessel = FlightGlobals.ActiveVessel;
            if (vessel == null || vessel.orbit == null)
            {
                SetStatus("Активный аппарат не найден.", true);
                return;
            }

            RouteLegResult firstLeg = bestResult.RouteLegs[0];
            if (firstLeg.To == null || firstLeg.CentralBody == null)
            {
                SetStatus("У первого участка нет корректной цели для манёвра.", true);
                return;
            }

            CelestialBody vesselBody = vessel.orbit.referenceBody != null ? vessel.orbit.referenceBody : vessel.mainBody;
            if (vesselBody != firstLeg.From)
            {
                SetStatus("Активный аппарат должен быть на орбите стартового тела: " + firstLeg.From.bodyName + ".", true);
                return;
            }

            if (firstLeg.CentralBody == null || !IsFinite(firstLeg.CentralBody.gravParameter) || firstLeg.CentralBody.gravParameter <= 0.0)
            {
                SetStatus("Автоузел не смог определить центр притяжения для первого участка.", true);
                return;
            }

            if (!vessel.PatchedConicsAttached)
            {
                vessel.AttachPatchedConicsSolver();
            }

            PatchedConicSolver solver = vessel.patchedConicSolver;
            if (solver == null)
            {
                SetStatus("KSP не дал доступ к планировщику манёвров активного аппарата.", true);
                return;
            }

            solver.maxTotalPatches = Math.Max(solver.maxTotalPatches, 12);
            RemoveExistingManeuverNodes(solver);

            ManeuverNode node = solver.AddManeuverNode(Math.Max(firstLeg.DepartUT, Planetarium.GetUniversalTime() + 60.0));
            RouteLegResult nextLeg = bestResult.RouteLegs.Length > 1 ? bestResult.RouteLegs[1] : null;
            ManeuverCandidate candidate = OptimizeManeuverNodeForFirstLeg(solver, node, vessel, firstLeg, nextLeg);
            if (candidate == null || !candidate.IsValid)
            {
                solver.RemoveManeuverNode(node);
                SetStatus("Не удалось построить игровой манёвр по текущей орбите аппарата.", true);
                return;
            }

            if (!IsManeuverCandidateGoodEnough(candidate, firstLeg, nextLeg))
            {
                solver.RemoveManeuverNode(node);
                SetStatus("Точный манёвр не создан: KSP не подтвердил вход в SOI/нужный перицентр по текущей орбите аппарата. Пересчитай маршрут с текущего аппарата или выбери качество «Точно».", true);
                return;
            }

            ApplyManeuverNode(node, candidate.UT, candidate.DeltaV);
            solver.UpdateFlightPlan();

            string targetName = firstLeg.To.bodyName;
            if (candidate.EncounterFound || candidate.MissDistance <= candidate.TargetSoi)
            {
                SetStatus(
                    "Манёвр подогнан по игре: " + targetName +
                    ", старт " + FormatUT(candidate.UT) +
                    ", Δv " + FormatDV(candidate.DeltaVMagnitude) +
                    ", промах " + FormatDistance(candidate.MissDistance) + ".",
                    true);
            }
            else
            {
                SetStatus(
                    "Манёвр создан, но KSP ещё не показывает вход в SOI " + targetName +
                    ". Минимальный промах: " + FormatDistance(candidate.MissDistance) +
                    " при SOI " + FormatDistance(candidate.TargetSoi) + ".",
                    true);
            }
        }

        private void SetStatus(string message, bool showScreenMessage)
        {
            statusText = message;
            if (showScreenMessage)
            {
                ScreenMessages.PostScreenMessage(message, 5f, ScreenMessageStyle.UPPER_CENTER);
            }
        }

        private static void RemoveExistingManeuverNodes(PatchedConicSolver solver)
        {
            if (solver == null || solver.maneuverNodes == null)
            {
                return;
            }

            for (int i = solver.maneuverNodes.Count - 1; i >= 0; i--)
            {
                solver.RemoveManeuverNode(solver.maneuverNodes[i]);
            }
            solver.UpdateFlightPlan();
        }

        private ManeuverCandidate OptimizeManeuverNodeForFirstLeg(PatchedConicSolver solver, ManeuverNode node, Vessel vessel, RouteLegResult leg, RouteLegResult nextLeg)
        {
            double now = Planetarium.GetUniversalTime() + 60.0;
            double plannedUT = Math.Max(leg.DepartUT, now);
            double orbitPeriod = vessel.orbit.period;
            if (!IsFinite(orbitPeriod) || orbitPeriod <= 1.0 || orbitPeriod > YearSeconds())
            {
                orbitPeriod = DaySeconds();
            }

            ManeuverCandidate best = null;
            double expectedDeltaV = leg.StartsFromActiveVessel && IsFinite(leg.ActiveVesselDepartureDV) && leg.ActiveVesselDepartureDV > 0.0
                ? leg.ActiveVesselDepartureDV
                : EstimateCurrentOrbitEjectionDeltaV(vessel.orbit, leg.From, leg.DepartureVInf, plannedUT);
            double maxDeltaV = IsFinite(expectedDeltaV) && expectedDeltaV > 0.0
                ? Math.Max(expectedDeltaV + 3000.0, expectedDeltaV * 4.0)
                : 5000.0;
            double startUT = Math.Max(now, plannedUT - orbitPeriod * 0.5);
            int timeSamples = qualityIndex <= 0 ? 48 : (qualityIndex == 1 ? 72 : 120);
            for (int i = 0; i <= timeSamples; i++)
            {
                double ut = startUT + orbitPeriod * i / timeSamples;
                TryEvaluateLambertSeed(solver, node, vessel, leg, nextLeg, ut, leg.LongWay, maxDeltaV, ref best);
                TryEvaluateLambertSeed(solver, node, vessel, leg, nextLeg, ut, !leg.LongWay, maxDeltaV, ref best);
            }

            if (best == null || !best.IsValid)
            {
                return best;
            }

            ApplyManeuverNode(node, best.UT, best.DeltaV);
            RefineManeuverNodeTime(solver, node, vessel, leg, nextLeg, orbitPeriod, maxDeltaV, ref best);
            RefineManeuverNodeComponents(solver, node, leg, nextLeg, maxDeltaV, ref best);
            if (best != null && IsFinite(best.MissDistance) && best.MissDistance > Math.Max(1.0, best.TargetSoi))
            {
                RefineManeuverNodeWideTime(solver, node, vessel, leg, nextLeg, orbitPeriod, maxDeltaV, ref best);
                RefineManeuverNodeComponents(solver, node, leg, nextLeg, maxDeltaV, ref best);
            }
            if (!IsManeuverCandidateGoodEnough(best, leg, nextLeg))
            {
                RefineManeuverNodeTime(solver, node, vessel, leg, nextLeg, orbitPeriod, maxDeltaV, ref best);
                RefineManeuverNodeComponents(solver, node, leg, nextLeg, maxDeltaV, ref best);
            }
            return best;
        }

        private void RefineManeuverNodeWideTime(PatchedConicSolver solver, ManeuverNode node, Vessel vessel, RouteLegResult leg, RouteLegResult nextLeg, double orbitPeriod, double maxDeltaV, ref ManeuverCandidate best)
        {
            if (best == null || !best.IsValid || vessel == null || vessel.orbit == null)
            {
                return;
            }

            double now = Planetarium.GetUniversalTime() + 60.0;
            double window = Math.Max(60.0, orbitPeriod);
            double[] offsets = { -0.75, -0.50, -0.25, -0.125, 0.125, 0.25, 0.50, 0.75 };
            ManeuverCandidate seedBest = best;
            for (int i = 0; i < offsets.Length; i++)
            {
                double testUT = Math.Max(now, seedBest.UT + window * offsets[i]);
                TryEvaluateLambertSeed(solver, node, vessel, leg, nextLeg, testUT, leg.LongWay, maxDeltaV, ref best);
                TryEvaluateLambertSeed(solver, node, vessel, leg, nextLeg, testUT, !leg.LongWay, maxDeltaV, ref best);
            }

            ApplyManeuverNode(node, best.UT, best.DeltaV);
        }

        private void RefineManeuverNodeTime(PatchedConicSolver solver, ManeuverNode node, Vessel vessel, RouteLegResult leg, RouteLegResult nextLeg, double orbitPeriod, double maxDeltaV, ref ManeuverCandidate best)
        {
            if (best == null || !best.IsValid)
            {
                return;
            }

            double now = Planetarium.GetUniversalTime() + 60.0;
            double[] steps =
            {
                orbitPeriod / 24.0,
                orbitPeriod / 72.0,
                orbitPeriod / 144.0,
                orbitPeriod / 432.0,
                orbitPeriod / 1296.0
            };

            for (int stepIndex = 0; stepIndex < steps.Length; stepIndex++)
            {
                double step = Math.Max(1.0, steps[stepIndex]);
                bool improved = true;
                int guard = 0;
                while (improved && guard < 10)
                {
                    improved = false;
                    guard++;
                    ManeuverCandidate before = best;
                    for (int sign = -1; sign <= 1; sign += 2)
                    {
                        double testUT = Math.Max(now, before.UT + sign * step);
                        TryEvaluateLambertSeed(solver, node, vessel, leg, nextLeg, testUT, leg.LongWay, maxDeltaV, ref best);
                        TryEvaluateLambertSeed(solver, node, vessel, leg, nextLeg, testUT, !leg.LongWay, maxDeltaV, ref best);
                    }

                    improved = best != before;
                }
            }

            ApplyManeuverNode(node, best.UT, best.DeltaV);
        }

        private void TryEvaluateLambertSeed(PatchedConicSolver solver, ManeuverNode node, Vessel vessel, RouteLegResult leg, RouteLegResult nextLeg, double ut, bool longWay, double maxDeltaV, ref ManeuverCandidate best)
        {
            double arrivalUT = leg.ArrivalUT + (ut - leg.DepartUT);
            List<Vector3d> seeds = new List<Vector3d>();

            Vector3d ejectionDeltaV;
            if (TryComputePlanetCenteredEjectionDeltaV(vessel, leg, ut, longWay, out ejectionDeltaV, out arrivalUT))
            {
                AddManeuverSeedVariants(seeds, ejectionDeltaV);
            }

            if (seeds.Count == 0)
            {
                return;
            }

            for (int i = 0; i < seeds.Count; i++)
            {
                if (seeds[i].magnitude > maxDeltaV)
                {
                    continue;
                }

                ApplyManeuverNode(node, ut, seeds[i]);
                ManeuverCandidate candidate = EvaluateCurrentManeuverNode(solver, node, leg, nextLeg, arrivalUT);
                if (candidate == null || !candidate.IsValid)
                {
                    continue;
                }

                candidate.DeltaV = seeds[i];
                candidate.UT = ut;
                candidate.ArrivalUT = arrivalUT;
                if (IsBetterManeuverCandidate(candidate, best))
                {
                    best = candidate;
                }
            }
        }

        private static void AddManeuverSeedVariants(List<Vector3d> seeds, Vector3d deltaV)
        {
            if (seeds == null || !IsFinite(deltaV.magnitude) || deltaV.magnitude <= 0.0 || deltaV.magnitude > 25000.0)
            {
                return;
            }

            for (int i = 0; i < ManeuverSeedRules.SignMasks.Length; i++)
            {
                SignMask mask = ManeuverSeedRules.SignMasks[i];
                AddManeuverSeed(seeds, new Vector3d(deltaV.x * mask.X, deltaV.y * mask.Y, deltaV.z * mask.Z));
            }
        }

        private static void AddManeuverSeed(List<Vector3d> seeds, Vector3d deltaV)
        {
            for (int i = 0; i < seeds.Count; i++)
            {
                if ((seeds[i] - deltaV).sqrMagnitude < 0.01)
                {
                    return;
                }
            }

            seeds.Add(deltaV);
        }

        private bool TryComputePlanetCenteredEjectionDeltaV(Vessel vessel, RouteLegResult leg, double nodeUT, bool longWay, out Vector3d deltaV, out double arrivalUT)
        {
            deltaV = Vector3d.zero;
            arrivalUT = leg.ArrivalUT + (nodeUT - leg.DepartUT);

            double minArrivalUT = nodeUT + DaySeconds() * 0.25;
            if (arrivalUT < minArrivalUT)
            {
                arrivalUT = minArrivalUT;
            }

            double tof = arrivalUT - nodeUT;
            if (!IsFinite(tof) || tof <= DaySeconds() * 0.1)
            {
                return false;
            }

            CelestialBody central = leg.CentralBody;
            Vector3d originPosition = GetBodyPositionRelativeTo(leg.From, central, nodeUT);
            Vector3d originVelocity = GetBodyVelocityRelativeTo(leg.From, central, nodeUT);
            Vector3d targetPosition = GetLegTargetPositionRelativeToCentral(leg, central, arrivalUT);

            if (leg.Kind == LegKind.ParentToChild && vessel != null && vessel.orbit != null && vessel.orbit.referenceBody == leg.From)
            {
                Vector3d vesselPosition = vessel.orbit.getRelativePositionAtUT(nodeUT);
                Vector3d vesselVelocity = vessel.orbit.getOrbitalVelocityAtUT(nodeUT);
                Vector3d directTransferVelocity;
                Vector3d directIgnoredArrivalVelocity;
                if (!LambertSolver.TrySolve(originPosition + vesselPosition, targetPosition, tof, central.gravParameter, longWay, out directTransferVelocity, out directIgnoredArrivalVelocity))
                {
                    return false;
                }

                Vector3d worldDelta = directTransferVelocity - (originVelocity + vesselVelocity);
                if (!IsFinite(worldDelta.magnitude) || worldDelta.magnitude <= 0.0 || worldDelta.magnitude > 25000.0)
                {
                    return false;
                }

                deltaV = WorldDeltaToManeuverDelta(vessel.orbit, nodeUT, worldDelta);
                return IsFinite(deltaV.magnitude);
            }

            Vector3d transferVelocity;
            Vector3d ignoredArrivalVelocity;
            if (!LambertSolver.TrySolve(originPosition, targetPosition, tof, central.gravParameter, longWay, out transferVelocity, out ignoredArrivalVelocity))
            {
                return false;
            }

            Vector3d desiredVInf = transferVelocity - originVelocity;
            if (!IsFinite(desiredVInf.magnitude) || desiredVInf.magnitude <= 0.0)
            {
                return false;
            }

            return TryComputeEjectionBurnFromVInf(vessel.orbit, leg.From, nodeUT, desiredVInf, out deltaV);
        }

        private static double EstimateCurrentOrbitEjectionDeltaV(Orbit parkingOrbit, CelestialBody origin, double vinf, double ut)
        {
            if (parkingOrbit == null || origin == null || !IsFinite(vinf) || vinf <= 0.0)
            {
                return double.PositiveInfinity;
            }

            double radius = parkingOrbit.getRelativePositionAtUT(ut).magnitude;
            if (!IsFinite(radius) || radius <= origin.Radius)
            {
                radius = Math.Abs(parkingOrbit.semiMajorAxis);
            }
            if (!IsFinite(radius) || radius <= origin.Radius)
            {
                return double.PositiveInfinity;
            }

            double mu = origin.gravParameter;
            double circularSpeed = Math.Sqrt(mu / radius);
            double hyperbolaSpeed = Math.Sqrt(vinf * vinf + 2.0 * mu / radius);
            return Math.Max(0.0, hyperbolaSpeed - circularSpeed);
        }

        private static bool TryComputeEjectionBurnFromVInf(Orbit parkingOrbit, CelestialBody origin, double nodeUT, Vector3d desiredVInf, out Vector3d deltaV)
        {
            deltaV = Vector3d.zero;
            if (parkingOrbit == null || origin == null || desiredVInf.sqrMagnitude < 1e-8)
            {
                return false;
            }

            Vector3d radius = parkingOrbit.getRelativePositionAtUT(nodeUT);
            Vector3d velocity = parkingOrbit.getOrbitalVelocityAtUT(nodeUT);
            if (radius.sqrMagnitude < 1e-8 || velocity.sqrMagnitude < 1e-8)
            {
                return false;
            }

            double r = Math.Max(origin.Radius + 1.0, radius.magnitude);
            double vinf = desiredVInf.magnitude;
            double mu = origin.gravParameter;
            double circularSpeed = velocity.magnitude;
            double hyperbolaSpeed = Math.Sqrt(vinf * vinf + 2.0 * mu / r);
            double eccentricity = 1.0 + r * vinf * vinf / mu;
            double inverseEccentricity = Clamp(1.0 / Math.Max(1.000001, eccentricity), 0.0, 0.999999);

            Vector3d radial = radius.normalized;
            Vector3d asymptote = desiredVInf.normalized;
            double radialAsymptoteDot = Vector3d.Dot(asymptote, radial);
            double outgoingAsymptoteDot = -inverseEccentricity;
            if (Math.Abs(radialAsymptoteDot - outgoingAsymptoteDot) > 0.30)
            {
                return false;
            }

            Vector3d periapsisVelocityDirection = asymptote - radial * radialAsymptoteDot;
            periapsisVelocityDirection -= radial * Vector3d.Dot(periapsisVelocityDirection, radial);
            if (periapsisVelocityDirection.sqrMagnitude < 1e-8)
            {
                periapsisVelocityDirection = velocity - radial * Vector3d.Dot(velocity, radial);
            }
            if (periapsisVelocityDirection.sqrMagnitude < 1e-8)
            {
                return false;
            }

            Vector3d requiredLocalVelocity = periapsisVelocityDirection.normalized * hyperbolaSpeed;
            Vector3d localBurn = requiredLocalVelocity - velocity;
            double idealBurn = Math.Max(0.0, hyperbolaSpeed - circularSpeed);
            double progradeAlignment = Vector3d.Dot(requiredLocalVelocity.normalized, velocity.normalized);
            if (progradeAlignment < 0.20 || localBurn.magnitude > idealBurn + Math.Max(350.0, idealBurn * 0.45))
            {
                return false;
            }

            if (!IsFinite(localBurn.magnitude) || localBurn.magnitude <= 0.0 || localBurn.magnitude > 25000.0)
            {
                return false;
            }

            deltaV = WorldDeltaToManeuverDelta(parkingOrbit, nodeUT, localBurn);
            return IsFinite(deltaV.magnitude);
        }

        private bool TryComputeGameStateLambertDeltaV(Vessel vessel, RouteLegResult leg, double nodeUT, bool longWay, out Vector3d deltaV, out double arrivalUT)
        {
            deltaV = Vector3d.zero;
            arrivalUT = leg.ArrivalUT + (nodeUT - leg.DepartUT);
            double minArrivalUT = nodeUT + DaySeconds() * 0.25;
            if (arrivalUT < minArrivalUT)
            {
                arrivalUT = minArrivalUT;
            }

            double tof = arrivalUT - nodeUT;
            if (!IsFinite(tof) || tof <= DaySeconds() * 0.1)
            {
                return false;
            }

            CelestialBody central = leg.CentralBody;
            Vector3d originPosition = GetBodyPositionRelativeTo(leg.From, central, nodeUT);
            Vector3d originVelocity = GetBodyVelocityRelativeTo(leg.From, central, nodeUT);
            Vector3d vesselPosition = vessel.orbit.getRelativePositionAtUT(nodeUT);
            Vector3d vesselVelocity = vessel.orbit.getOrbitalVelocityAtUT(nodeUT);
            Vector3d targetPosition = GetLegTargetPositionRelativeToCentral(leg, central, arrivalUT);

            Vector3d r1 = originPosition + vesselPosition;
            Vector3d currentVelocity = originVelocity + vesselVelocity;
            Vector3d transferVelocity;
            Vector3d ignoredArrivalVelocity;
            if (!LambertSolver.TrySolve(r1, targetPosition, tof, central.gravParameter, longWay, out transferVelocity, out ignoredArrivalVelocity))
            {
                return false;
            }

            Vector3d worldDelta = transferVelocity - currentVelocity;
            if (!IsFinite(worldDelta.magnitude) || worldDelta.magnitude <= 0.0 || worldDelta.magnitude > 25000.0)
            {
                return false;
            }

            deltaV = WorldDeltaToManeuverDelta(vessel.orbit, nodeUT, worldDelta);
            return IsFinite(deltaV.magnitude);
        }

        private void RefineManeuverNodeComponents(PatchedConicSolver solver, ManeuverNode node, RouteLegResult leg, RouteLegResult nextLeg, double maxDeltaV, ref ManeuverCandidate best)
        {
            if (best == null || !best.IsValid)
            {
                return;
            }

            bool flybyTarget = nextLeg != null && leg != null && nextLeg.From == leg.To;
            double[] steps = BuildManeuverComponentSteps(best.DeltaVMagnitude, maxDeltaV, flybyTarget);
            for (int stepIndex = 0; stepIndex < steps.Length; stepIndex++)
            {
                double step = steps[stepIndex];
                bool improved = true;
                int guard = 0;
                while (improved && guard < 18)
                {
                    improved = false;
                    guard++;
                    for (int axis = 0; axis < 3; axis++)
                    {
                        for (int sign = -1; sign <= 1; sign += 2)
                        {
                            Vector3d testDelta = best.DeltaV;
                            if (axis == 0) testDelta.x += sign * step;
                            if (axis == 1) testDelta.y += sign * step;
                            if (axis == 2) testDelta.z += sign * step;
                            if (testDelta.magnitude > maxDeltaV)
                            {
                                continue;
                            }

                            ApplyManeuverNode(node, best.UT, testDelta);

                            ManeuverCandidate candidate = EvaluateCurrentManeuverNode(solver, node, leg, nextLeg, best.ArrivalUT);
                            if (candidate == null || !candidate.IsValid)
                            {
                                continue;
                            }

                            candidate.DeltaV = testDelta;
                            candidate.UT = best.UT;
                            candidate.ArrivalUT = best.ArrivalUT;
                            if (IsBetterManeuverCandidate(candidate, best))
                            {
                                best = candidate;
                                improved = true;
                            }
                        }
                    }
                }

                if (IsManeuverCandidateGoodEnough(best, leg, nextLeg))
                {
                    break;
                }
            }

            ApplyManeuverNode(node, best.UT, best.DeltaV);
        }

        private static double[] BuildManeuverComponentSteps(double currentDeltaV, double maxDeltaV, bool flybyTarget)
        {
            double scale = IsFinite(currentDeltaV) && currentDeltaV > 0.0
                ? currentDeltaV
                : (IsFinite(maxDeltaV) && maxDeltaV > 0.0 ? maxDeltaV * 0.5 : 1000.0);
            double first = Clamp(scale * (flybyTarget ? 0.55 : 0.70), 20.0, flybyTarget ? 1800.0 : 2400.0);
            return new[]
            {
                first,
                Math.Max(7.0, first * 0.45),
                Math.Max(2.0, first * 0.20),
                Math.Max(0.7, first * 0.085),
                Math.Max(0.2, first * 0.035),
                Math.Max(0.06, first * 0.014),
                Math.Max(0.015, first * 0.005),
                0.015
            };
        }

        private static bool IsManeuverCandidateGoodEnough(ManeuverCandidate candidate, RouteLegResult leg, RouteLegResult nextLeg)
        {
            if (candidate == null || !candidate.IsValid)
            {
                return false;
            }

            if (nextLeg != null && leg != null && nextLeg.From == leg.To)
            {
                double desiredPeriapsis = GetPlannedFlybyPeriapsisRadius(leg);
                return candidate.EncounterFound &&
                    IsManeuverPeriapsisAcceptable(leg.To, candidate.FlybyPeriapsisRadius, desiredPeriapsis, false);
            }

            if (leg != null && IsFinite(leg.PlannedCapturePeriapsisRadius) && leg.PlannedCapturePeriapsisRadius > 0.0)
            {
                double desiredPeriapsis = GetPlannedCapturePeriapsisRadius(leg);
                return candidate.EncounterFound &&
                    IsManeuverPeriapsisAcceptable(leg.To, candidate.FlybyPeriapsisRadius, desiredPeriapsis, true);
            }

            return candidate.EncounterFound || candidate.MissDistance <= candidate.TargetSoi;
        }

        private static double GetPlannedFlybyPeriapsisRadius(RouteLegResult leg)
        {
            if (leg != null && IsFinite(leg.PlannedFlybyPeriapsisRadius) && leg.PlannedFlybyPeriapsisRadius > 0.0)
            {
                return leg.PlannedFlybyPeriapsisRadius;
            }

            return leg != null && leg.To != null ? SafeFlybyPeriapsisRadius(leg.To) : 1.0;
        }

        private static double GetPlannedCapturePeriapsisRadius(RouteLegResult leg)
        {
            if (leg != null && IsFinite(leg.PlannedCapturePeriapsisRadius) && leg.PlannedCapturePeriapsisRadius > 0.0)
            {
                return leg.PlannedCapturePeriapsisRadius;
            }

            return leg != null && leg.To != null ? GetCapturePeriapsisRadius(leg.To, 0.0) : 1.0;
        }

        private static bool IsManeuverPeriapsisAcceptable(CelestialBody body, double actualPeriapsisRadius, double desiredPeriapsisRadius, bool strictCapture)
        {
            if (body == null || !IsFinite(actualPeriapsisRadius) || !IsFinite(desiredPeriapsisRadius) || actualPeriapsisRadius <= 0.0 || desiredPeriapsisRadius <= 0.0)
            {
                return false;
            }

            double safeAltitude = GetMinimumSafeOrbitAltitude(body);
            double safeRadius = body.Radius + safeAltitude;
            if (actualPeriapsisRadius < safeRadius)
            {
                return false;
            }

            double desiredAltitude = Math.Max(0.0, desiredPeriapsisRadius - body.Radius);
            double bodyScaleTolerance = strictCapture
                ? Math.Min(body.Radius * 0.02, 100000.0)
                : Math.Min(body.Radius * 0.20, desiredPeriapsisRadius * 0.75);
            double altitudeTolerance = strictCapture ? desiredAltitude * 0.35 : desiredAltitude;
            double minimumTolerance = strictCapture ? 25000.0 : 50000.0;
            double tolerance = Math.Max(minimumTolerance, Math.Max(altitudeTolerance, bodyScaleTolerance));

            return Math.Abs(actualPeriapsisRadius - desiredPeriapsisRadius) <= tolerance;
        }

        private ManeuverCandidate EvaluateCurrentManeuverNode(PatchedConicSolver solver, ManeuverNode node, RouteLegResult leg, RouteLegResult nextLeg, double arrivalUT)
        {
            if (solver == null || node == null || leg == null || leg.To == null)
            {
                return null;
            }

            solver.UpdateFlightPlan();
            ManeuverCandidate candidate = new ManeuverCandidate
            {
                IsValid = true,
                UT = node.UT,
                DeltaV = node.DeltaV,
                ArrivalUT = arrivalUT,
                MissDistance = double.PositiveInfinity,
                TargetSoi = GetSafeSphereOfInfluence(leg.To),
                RequiresEncounter = nextLeg != null && nextLeg.From == leg.To,
                Score = double.PositiveInfinity,
                IncomingVInfError = double.PositiveInfinity,
                OutgoingVInfError = double.PositiveInfinity,
                FlybyPeriapsisRadius = double.PositiveInfinity,
                NextTargetMissDistance = double.PositiveInfinity,
                NextTargetSoi = nextLeg != null ? GetSafeSphereOfInfluence(nextLeg.To) : 1.0
            };

            List<Orbit> patches = CollectSolverPatches(solver, node);
            Orbit targetPatch = null;
            for (int i = 0; i < patches.Count; i++)
            {
                Orbit patch = patches[i];
                if (patch == null)
                {
                    continue;
                }

                if (patch.referenceBody == leg.To)
                {
                    candidate.EncounterFound = true;
                    candidate.MissDistance = 0.0;
                    candidate.EncounterUT = patch.StartUT;
                    targetPatch = patch;
                    break;
                }

                if (patch.closestEncounterBody == leg.To && patch.closestEncounterPatch != null && patch.closestEncounterPatch.referenceBody == leg.To)
                {
                    candidate.EncounterFound = true;
                    candidate.MissDistance = 0.0;
                    candidate.EncounterUT = patch.closestEncounterPatch.StartUT;
                    targetPatch = patch.closestEncounterPatch;
                    break;
                }
            }

            if (!candidate.EncounterFound)
            {
                for (int i = 0; i < patches.Count; i++)
                {
                    Orbit patch = patches[i];
                    if (patch == null || patch.referenceBody != leg.CentralBody)
                    {
                        continue;
                    }

                    double missUT;
                    double miss = SampleClosestApproach(patch, leg.To, leg.CentralBody, arrivalUT, out missUT);
                    if (IsFinite(miss) && miss < candidate.MissDistance)
                    {
                        candidate.MissDistance = miss;
                        candidate.EncounterUT = missUT;
                    }
                }
            }
            else if (targetPatch != null)
            {
                candidate.FlybyPeriapsisRadius = EstimatePatchPeriapsisRadius(targetPatch, leg.To, candidate.EncounterUT);
            }

            ScoreManeuverCandidate(candidate, patches, targetPatch, leg, nextLeg, arrivalUT);
            return IsFinite(candidate.MissDistance) ? candidate : null;
        }

        private static void ScoreManeuverCandidate(ManeuverCandidate candidate, List<Orbit> patches, Orbit targetPatch, RouteLegResult leg, RouteLegResult nextLeg, double plannedArrivalUT)
        {
            if (candidate == null || leg == null || leg.To == null)
            {
                return;
            }

            double targetSoi = Math.Max(1.0, candidate.TargetSoi);
            double missScore = candidate.MissDistance / targetSoi;
            if (candidate.EncounterFound)
            {
                missScore = 0.0;
            }
            double encounterPenalty = candidate.EncounterFound ? 0.0 : 60.0;

            double encounterUT = IsFinite(candidate.EncounterUT) && candidate.EncounterUT > 0.0 ? candidate.EncounterUT : plannedArrivalUT;
            double timeTolerance = Math.Max(DaySeconds() * 0.25, Math.Min(DaySeconds() * 12.0, Math.Max(DaySeconds(), leg.TimeOfFlight * 0.01)));
            double timeScore = Math.Abs(encounterUT - plannedArrivalUT) / timeTolerance;

            Vector3d incomingVInf;
            if (TryEstimateCentralRelativeVelocity(patches, leg.CentralBody, leg.To, encounterUT, out incomingVInf))
            {
                candidate.IncomingVInfError = RelativeVectorError(incomingVInf, leg.ArrivalVInfVector);
            }

            double outgoingScore = 0.0;
            double flybyPeriapsisScore = 0.0;
            if (nextLeg != null && nextLeg.From == leg.To)
            {
                double desiredPeriapsis = GetPlannedFlybyPeriapsisRadius(leg);
                if (IsFinite(candidate.FlybyPeriapsisRadius))
                {
                    double highMiss = Math.Max(0.0, candidate.FlybyPeriapsisRadius - desiredPeriapsis) / Math.Max(1.0, desiredPeriapsis);
                    double unsafeMiss = Math.Max(0.0, leg.To.Radius + GetMinimumSafeOrbitAltitude(leg.To) - candidate.FlybyPeriapsisRadius) / Math.Max(1.0, desiredPeriapsis);
                    flybyPeriapsisScore = highMiss * highMiss + highMiss * 0.25 + unsafeMiss * 30.0;
                }
                else
                {
                    flybyPeriapsisScore = candidate.EncounterFound ? 25.0 : 6.0;
                }

                Vector3d outgoingVInf;
                if (TryEstimatePostEncounterVInf(patches, targetPatch, leg.CentralBody, leg.To, out outgoingVInf))
                {
                    candidate.OutgoingVInfError = RelativeVectorError(outgoingVInf, nextLeg.DepartureVInfVector);
                    outgoingScore = candidate.OutgoingVInfError;
                }
                else
                {
                    outgoingScore = candidate.EncounterFound ? 8.0 : 2.0;
                }

                Orbit postPatch;
                if (TryFindPostEncounterPatch(patches, targetPatch, leg.CentralBody, out postPatch))
                {
                    double nextMissUT;
                    double nextMiss = SampleClosestApproach(postPatch, nextLeg.To, leg.CentralBody, nextLeg.ArrivalUT, out nextMissUT);
                    if (IsFinite(nextMiss))
                    {
                        candidate.NextTargetMissDistance = nextMiss;
                    }
                }

                for (int i = 0; patches != null && i < patches.Count; i++)
                {
                    Orbit patch = patches[i];
                    if (patch != null && patch.referenceBody == nextLeg.To)
                    {
                        candidate.NextTargetMissDistance = 0.0;
                        break;
                    }
                }
            }
            else if (IsFinite(leg.PlannedCapturePeriapsisRadius) && leg.PlannedCapturePeriapsisRadius > 0.0)
            {
                double desiredPeriapsis = GetPlannedCapturePeriapsisRadius(leg);
                if (IsFinite(candidate.FlybyPeriapsisRadius))
                {
                    double periapsisMiss = Math.Abs(candidate.FlybyPeriapsisRadius - desiredPeriapsis) / Math.Max(1.0, desiredPeriapsis);
                    double unsafeMiss = Math.Max(0.0, leg.To.Radius + GetMinimumSafeOrbitAltitude(leg.To) - candidate.FlybyPeriapsisRadius) / Math.Max(1.0, desiredPeriapsis);
                    flybyPeriapsisScore = periapsisMiss * periapsisMiss + periapsisMiss * 0.25 + unsafeMiss * 30.0;
                }
                else
                {
                    flybyPeriapsisScore = candidate.EncounterFound ? 25.0 : 6.0;
                }
            }

            double incomingScore = IsFinite(candidate.IncomingVInfError) ? candidate.IncomingVInfError : 2.0;
            double nextTargetScore = 0.0;
            if (nextLeg != null && nextLeg.From == leg.To)
            {
                nextTargetScore = IsFinite(candidate.NextTargetMissDistance)
                    ? candidate.NextTargetMissDistance / Math.Max(1.0, candidate.NextTargetSoi)
                    : 20.0;
            }

            candidate.Score =
                encounterPenalty +
                Clamp(missScore, 0.0, 20.0) * 8.0 +
                Clamp(timeScore, 0.0, 20.0) * 1.5 +
                Clamp(incomingScore, 0.0, 20.0) * 2.0 +
                Clamp(flybyPeriapsisScore, 0.0, 80.0) * 20.0 +
                Clamp(nextTargetScore, 0.0, 80.0) * 30.0 +
                Clamp(outgoingScore, 0.0, 20.0) * 5.0;
        }

        private static bool TryEstimateCentralRelativeVelocity(List<Orbit> patches, CelestialBody central, CelestialBody target, double ut, out Vector3d relativeVelocity)
        {
            relativeVelocity = Vector3d.zero;
            if (patches == null || central == null || target == null || !IsFinite(ut))
            {
                return false;
            }

            Orbit bestPatch = null;
            for (int i = 0; i < patches.Count; i++)
            {
                Orbit patch = patches[i];
                if (patch == null || patch.referenceBody != central)
                {
                    continue;
                }

                double start = patch.StartUT;
                double end = patch.EndUT;
                if ((!IsFinite(start) || ut >= start - 5.0) && (!IsFinite(end) || end <= 0.0 || ut <= end + 5.0))
                {
                    bestPatch = patch;
                    break;
                }
            }

            if (bestPatch == null)
            {
                return false;
            }

            relativeVelocity = bestPatch.getOrbitalVelocityAtUT(ut) - GetBodyVelocityRelativeTo(target, central, ut);
            return IsFinite(relativeVelocity.magnitude);
        }

        private static bool TryEstimatePostEncounterVInf(List<Orbit> patches, Orbit targetPatch, CelestialBody central, CelestialBody target, out Vector3d relativeVelocity)
        {
            relativeVelocity = Vector3d.zero;
            if (central == null || target == null)
            {
                return false;
            }

            Orbit postPatch = null;
            if (!TryFindPostEncounterPatch(patches, targetPatch, central, out postPatch))
            {
                return false;
            }

            double ut = postPatch.StartUT;
            if (!IsFinite(ut) || ut <= 0.0)
            {
                ut = targetPatch != null && IsFinite(targetPatch.EndUT) ? targetPatch.EndUT : 0.0;
            }
            if (!IsFinite(ut) || ut <= 0.0)
            {
                return false;
            }

            relativeVelocity = postPatch.getOrbitalVelocityAtUT(ut) - GetBodyVelocityRelativeTo(target, central, ut);
            return IsFinite(relativeVelocity.magnitude);
        }

        private static bool TryFindPostEncounterPatch(List<Orbit> patches, Orbit targetPatch, CelestialBody central, out Orbit postPatch)
        {
            postPatch = null;
            if (central == null)
            {
                return false;
            }

            if (targetPatch != null)
            {
                Orbit patch = targetPatch.nextPatch;
                int guard = 0;
                while (patch != null && guard < 8)
                {
                    if (patch.referenceBody == central)
                    {
                        postPatch = patch;
                        return true;
                    }
                    patch = patch.nextPatch;
                    guard++;
                }
            }

            if (patches != null && targetPatch != null)
            {
                int targetIndex = patches.IndexOf(targetPatch);
                for (int i = Math.Max(0, targetIndex + 1); i < patches.Count; i++)
                {
                    Orbit patch = patches[i];
                    if (patch != null && patch.referenceBody == central)
                    {
                        postPatch = patch;
                        return true;
                    }
                }
            }

            return false;
        }

        private static double RelativeVectorError(Vector3d actual, Vector3d expected)
        {
            double expectedMagnitude = expected.magnitude;
            double actualMagnitude = actual.magnitude;
            if (!IsFinite(expectedMagnitude) || !IsFinite(actualMagnitude) || expectedMagnitude <= 1e-3 || actualMagnitude <= 1e-3)
            {
                return double.PositiveInfinity;
            }

            double magnitudeError = Math.Abs(actualMagnitude - expectedMagnitude) / Math.Max(1.0, expectedMagnitude);
            double directionError = AngleRad(actual, expected) / Math.PI;
            return magnitudeError + directionError;
        }

        private static double EstimatePatchPeriapsisRadius(Orbit patch, CelestialBody body, double encounterUT)
        {
            if (patch == null || body == null)
            {
                return double.PositiveInfinity;
            }

            double peR = patch.PeR;
            if (IsFinite(peR) && peR > 0.0)
            {
                return peR;
            }

            double startUT = patch.StartUT;
            if (!IsFinite(startUT) || startUT <= 0.0)
            {
                startUT = IsFinite(encounterUT) && encounterUT > 0.0 ? encounterUT : Planetarium.GetUniversalTime();
            }

            double endUT = patch.EndUT;
            if (!IsFinite(endUT) || endUT <= startUT)
            {
                double speed = patch.getOrbitalVelocityAtUT(startUT).magnitude;
                double soiCrossingTime = IsFinite(speed) && speed > 1.0 ? GetSafeSphereOfInfluence(body) / speed : DaySeconds() * 20.0;
                endUT = startUT + Math.Max(DaySeconds(), Math.Min(DaySeconds() * 80.0, soiCrossingTime * 2.0));
            }

            double bestRadius = double.PositiveInfinity;
            double bestUT = startUT;
            const int samples = 96;
            for (int i = 0; i <= samples; i++)
            {
                double ut = startUT + (endUT - startUT) * i / samples;
                double radius = patch.getRelativePositionAtUT(ut).magnitude;
                if (IsFinite(radius) && radius < bestRadius)
                {
                    bestRadius = radius;
                    bestUT = ut;
                }
            }

            double refineSpan = (endUT - startUT) / samples;
            for (int pass = 0; pass < 4; pass++)
            {
                double localStart = Math.Max(startUT, bestUT - refineSpan);
                double localEnd = Math.Min(endUT, bestUT + refineSpan);
                for (int i = 0; i <= 12; i++)
                {
                    double ut = localStart + (localEnd - localStart) * i / 12.0;
                    double radius = patch.getRelativePositionAtUT(ut).magnitude;
                    if (IsFinite(radius) && radius < bestRadius)
                    {
                        bestRadius = radius;
                        bestUT = ut;
                    }
                }
                refineSpan *= 0.25;
            }

            return bestRadius;
        }

        private static List<Orbit> CollectSolverPatches(PatchedConicSolver solver, ManeuverNode node)
        {
            List<Orbit> patches = new List<Orbit>();
            AddPatchIfMissing(patches, node.patch);
            AddPatchChain(patches, node.nextPatch);
            if (solver.flightPlan != null)
            {
                for (int i = 0; i < solver.flightPlan.Count; i++)
                {
                    AddPatchIfMissing(patches, solver.flightPlan[i]);
                }
            }
            if (solver.patches != null)
            {
                for (int i = 0; i < solver.patches.Count; i++)
                {
                    AddPatchIfMissing(patches, solver.patches[i]);
                }
            }
            AddPatchChain(patches, solver.orbit);
            return patches;
        }

        private static void AddPatchChain(List<Orbit> patches, Orbit patch)
        {
            int guard = 0;
            while (patch != null && guard < 16)
            {
                AddPatchIfMissing(patches, patch);
                patch = patch.nextPatch;
                guard++;
            }
        }

        private static void AddPatchIfMissing(List<Orbit> patches, Orbit patch)
        {
            if (patch != null && !patches.Contains(patch))
            {
                patches.Add(patch);
            }
        }

        private static double SampleClosestApproach(Orbit patch, CelestialBody target, CelestialBody central, double plannedArrivalUT, out double bestUT)
        {
            bestUT = plannedArrivalUT;
            double startUT = patch.StartUT;
            double endUT = patch.EndUT;
            if (!IsFinite(startUT) || startUT <= 0.0)
            {
                startUT = Planetarium.GetUniversalTime();
            }
            if (!IsFinite(endUT) || endUT <= startUT)
            {
                endUT = plannedArrivalUT + Math.Max(DaySeconds() * 20.0, Math.Abs(plannedArrivalUT - startUT) * 0.2);
            }

            double window = Math.Max(DaySeconds() * 20.0, Math.Min(YearSeconds(), Math.Abs(plannedArrivalUT - startUT) * 0.25));
            double sampleStart = Math.Max(startUT, plannedArrivalUT - window);
            double sampleEnd = Math.Min(endUT, plannedArrivalUT + window);
            if (sampleEnd <= sampleStart)
            {
                sampleStart = startUT;
                sampleEnd = Math.Min(endUT, startUT + window * 2.0);
            }
            if (sampleEnd <= sampleStart)
            {
                return double.PositiveInfinity;
            }

            double bestMiss = double.PositiveInfinity;
            const int samples = 72;
            for (int i = 0; i <= samples; i++)
            {
                double ut = sampleStart + (sampleEnd - sampleStart) * i / samples;
                double miss = DistanceToTargetAtUT(patch, target, central, ut);
                if (miss < bestMiss)
                {
                    bestMiss = miss;
                    bestUT = ut;
                }
            }

            double refineSpan = (sampleEnd - sampleStart) / samples;
            for (int pass = 0; pass < 4; pass++)
            {
                double localStart = Math.Max(startUT, bestUT - refineSpan);
                double localEnd = Math.Min(endUT, bestUT + refineSpan);
                for (int i = 0; i <= 12; i++)
                {
                    double ut = localStart + (localEnd - localStart) * i / 12.0;
                    double miss = DistanceToTargetAtUT(patch, target, central, ut);
                    if (miss < bestMiss)
                    {
                        bestMiss = miss;
                        bestUT = ut;
                    }
                }
                refineSpan *= 0.25;
            }

            return bestMiss;
        }

        private static double DistanceToTargetAtUT(Orbit patch, CelestialBody target, CelestialBody central, double ut)
        {
            Vector3d vesselPosition = patch.getRelativePositionAtUT(ut);
            Vector3d targetPosition = GetBodyPositionRelativeTo(target, central, ut);
            return (vesselPosition - targetPosition).magnitude;
        }

        private static bool IsBetterManeuverCandidate(ManeuverCandidate candidate, ManeuverCandidate current)
        {
            if (candidate == null || !candidate.IsValid)
            {
                return false;
            }
            if (current == null || !current.IsValid)
            {
                return true;
            }

            if ((candidate.RequiresEncounter || current.RequiresEncounter) && candidate.EncounterFound != current.EncounterFound)
            {
                return candidate.EncounterFound;
            }

            if (IsFinite(candidate.Score) || IsFinite(current.Score))
            {
                if (!IsFinite(current.Score))
                {
                    return true;
                }
                if (!IsFinite(candidate.Score))
                {
                    return false;
                }
                if (Math.Abs(candidate.Score - current.Score) > 0.001)
                {
                    return candidate.Score < current.Score;
                }
            }

            if (candidate.EncounterFound != current.EncounterFound)
            {
                return candidate.EncounterFound;
            }

            double candidateScore = candidate.MissDistance / Math.Max(1.0, candidate.TargetSoi);
            double currentScore = current.MissDistance / Math.Max(1.0, current.TargetSoi);
            if (Math.Abs(candidateScore - currentScore) > 0.001)
            {
                return candidateScore < currentScore;
            }

            return candidate.DeltaVMagnitude < current.DeltaVMagnitude;
        }

        private static void ApplyManeuverNode(ManeuverNode node, double ut, Vector3d deltaV)
        {
            node.UT = ut;
            node.DeltaV = deltaV;
            node.OnGizmoUpdated(deltaV, ut);
        }

        private static Vector3d WorldDeltaToManeuverDelta(Orbit orbit, double ut, Vector3d worldDelta)
        {
            Vector3d radius = orbit.getRelativePositionAtUT(ut);
            Vector3d velocity = orbit.getOrbitalVelocityAtUT(ut);
            if (radius.sqrMagnitude < 1e-12 || velocity.sqrMagnitude < 1e-12)
            {
                return new Vector3d(0.0, 0.0, worldDelta.magnitude);
            }

            Vector3d prograde = velocity.normalized;
            Vector3d normal = Vector3d.Cross(radius, velocity).normalized;
            Vector3d radial = Vector3d.Cross(prograde, normal).normalized;
            return new Vector3d(
                Vector3d.Dot(worldDelta, radial),
                Vector3d.Dot(worldDelta, normal),
                Vector3d.Dot(worldDelta, prograde));
        }

        private static Vector3d ManeuverDeltaToWorldDelta(Orbit orbit, double ut, Vector3d maneuverDelta)
        {
            Vector3d radius = orbit.getRelativePositionAtUT(ut);
            Vector3d velocity = orbit.getOrbitalVelocityAtUT(ut);
            if (radius.sqrMagnitude < 1e-12 || velocity.sqrMagnitude < 1e-12)
            {
                return new Vector3d(0.0, 0.0, maneuverDelta.magnitude);
            }

            Vector3d prograde = velocity.normalized;
            Vector3d normal = Vector3d.Cross(radius, velocity).normalized;
            Vector3d radial = Vector3d.Cross(prograde, normal).normalized;
            return radial * maneuverDelta.x + normal * maneuverDelta.y + prograde * maneuverDelta.z;
        }

        private static bool TryGetLegGeometry(CelestialBody from, CelestialBody to, out LegGeometry geometry)
        {
            geometry = null;
            if (from == null || to == null || from == to || from.orbit == null || to.orbit == null ||
                from.orbit.referenceBody == null || to.orbit.referenceBody == null)
            {
                return false;
            }

            CelestialBody fromParent = from.orbit.referenceBody;
            CelestialBody toParent = to.orbit.referenceBody;
            CelestialBody central;
            LegKind kind;
            if (toParent == from)
            {
                central = from;
                kind = LegKind.ParentToChild;
            }
            else if (fromParent == to)
            {
                central = to;
                kind = LegKind.ChildToParent;
            }
            else if (fromParent == toParent)
            {
                central = fromParent;
                kind = LegKind.SameParent;
            }
            else
            {
                central = FindCommonTransferCentral(from, to);
                kind = LegKind.CommonAncestor;
            }

            if (central == null || !IsFinite(central.gravParameter) || central.gravParameter <= 0.0)
            {
                return false;
            }

            geometry = new LegGeometry
            {
                From = from,
                To = to,
                CentralBody = central,
                Kind = kind
            };
            return true;
        }

        private static CelestialBody FindCommonTransferCentral(CelestialBody from, CelestialBody to)
        {
            for (CelestialBody fromCursor = from; fromCursor != null; fromCursor = GetParentBody(fromCursor))
            {
                for (CelestialBody toCursor = to; toCursor != null; toCursor = GetParentBody(toCursor))
                {
                    if (fromCursor == toCursor)
                    {
                        return fromCursor;
                    }
                }
            }

            return null;
        }

        private static CelestialBody GetParentBody(CelestialBody body)
        {
            return body != null && body.orbit != null ? body.orbit.referenceBody : null;
        }

        private static bool CanConnectLegsByFreeFlyby(RouteLegResult incomingLeg, RouteLegResult outgoingLeg)
        {
            if (incomingLeg == null || outgoingLeg == null || incomingLeg.To == null || outgoingLeg.From == null)
            {
                return false;
            }

            return incomingLeg.To == outgoingLeg.From &&
                incomingLeg.CentralBody == outgoingLeg.CentralBody &&
                incomingLeg.To != incomingLeg.CentralBody &&
                outgoingLeg.From != outgoingLeg.CentralBody;
        }

        private static CelestialBody GetRouteSearchCentral(CelestialBody origin, List<MissionTarget> targets)
        {
            CelestialBody central = null;
            if (targets == null || targets.Count == 0)
            {
                return origin != null && origin.orbit != null ? origin.orbit.referenceBody : null;
            }

            for (int i = 0; i < targets.Count; i++)
            {
                LegGeometry geometry;
                if (!TryGetLegGeometry(origin, targets[i].Body, out geometry))
                {
                    continue;
                }

                if (central == null)
                {
                    central = geometry.CentralBody;
                }
                else if (central != geometry.CentralBody)
                {
                    CelestialBody common = FindCommonTransferCentral(central, geometry.CentralBody);
                    if (common != null)
                    {
                        central = common;
                    }
                }
            }

            return central;
        }

        private static double EstimateOrbitRadiusAroundCentral(CelestialBody body, CelestialBody central, double ut)
        {
            if (body == null || central == null)
            {
                return double.PositiveInfinity;
            }

            Vector3d position = GetBodyPositionRelativeTo(body, central, ut);
            double radius = position.magnitude;
            if (IsFinite(radius) && radius > 1.0)
            {
                return radius;
            }

            if (body.orbit != null && body.orbit.referenceBody == central)
            {
                radius = Math.Abs(body.orbit.semiMajorAxis);
                if (IsFinite(radius) && radius > 1.0)
                {
                    return radius;
                }
            }

            return double.PositiveInfinity;
        }

        private static bool TryGetTransferPlaneBasis(CelestialBody orbitingBody, CelestialBody central, double ut, out Vector3d xAxis, out Vector3d yAxis, out Vector3d normal)
        {
            xAxis = Vector3d.zero;
            yAxis = Vector3d.zero;
            normal = Vector3d.zero;
            if (orbitingBody == null || central == null)
            {
                return false;
            }

            Vector3d position = GetBodyPositionRelativeTo(orbitingBody, central, ut);
            Vector3d velocity = GetBodyVelocityRelativeTo(orbitingBody, central, ut);
            if (position.sqrMagnitude < 1e-8)
            {
                position = new Vector3d(1.0, 0.0, 0.0);
            }

            normal = SafeNormal(position, velocity);
            xAxis = position.normalized;
            yAxis = Vector3d.Cross(normal, xAxis);
            if (yAxis.sqrMagnitude < 1e-10)
            {
                Vector3d reference = Math.Abs(Vector3d.Dot(normal, new Vector3d(0.0, 1.0, 0.0))) < 0.85
                    ? new Vector3d(0.0, 1.0, 0.0)
                    : new Vector3d(1.0, 0.0, 0.0);
                xAxis = Vector3d.Cross(reference, normal).normalized;
                yAxis = Vector3d.Cross(normal, xAxis);
            }

            if (xAxis.sqrMagnitude < 1e-10 || yAxis.sqrMagnitude < 1e-10)
            {
                return false;
            }

            yAxis = yAxis.normalized;
            return true;
        }

        private int GetLocalOrbitPhaseSamples()
        {
            if (qualityIndex <= 0)
            {
                return 8;
            }

            return qualityIndex == 1 ? 12 : 18;
        }

        private static Vector3d GetBodyPositionRelativeTo(CelestialBody body, CelestialBody central, double ut)
        {
            return GetBodyLocalPositionAtUT(body, ut) - GetBodyLocalPositionAtUT(central, ut);
        }

        private static Vector3d GetLegTargetPositionRelativeToCentral(RouteLegResult leg, CelestialBody central, double ut)
        {
            if (leg != null && leg.Kind == LegKind.ChildToParent && leg.HasInsertionDVOverride)
            {
                return leg.EndPosition;
            }

            Vector3d position = GetBodyPositionRelativeTo(leg.To, central, ut);
            if (leg.HasTargetOffset)
            {
                position += leg.TargetOffset;
            }

            return position;
        }

        private static Vector3d GetBodyVelocityRelativeTo(CelestialBody body, CelestialBody central, double ut)
        {
            return GetBodyLocalVelocityAtUT(body, ut) - GetBodyLocalVelocityAtUT(central, ut);
        }

        private static Vector3d GetBodyLocalVelocityAtUT(CelestialBody body, double ut)
        {
            if (body == null || body.orbit == null)
            {
                return Vector3d.zero;
            }

            Vector3d velocity = body.orbit.getOrbitalVelocityAtUT(ut);
            CelestialBody parent = body.orbit.referenceBody;
            if (parent != null && parent.orbit != null)
            {
                velocity += GetBodyLocalVelocityAtUT(parent, ut);
            }

            return velocity;
        }

        private static double GetSafeSphereOfInfluence(CelestialBody body)
        {
            if (body == null)
            {
                return 1.0;
            }

            double soi = body.sphereOfInfluence;
            if (!IsFinite(soi) || soi <= 0.0)
            {
                soi = body.Radius * 10.0;
            }
            return Math.Max(1.0, soi);
        }

        private static string FormatDistance(double meters)
        {
            if (!IsFinite(meters))
            {
                return "—";
            }
            if (meters >= 1000000000.0)
            {
                return FormatNumber(meters / 1000000000.0, 2) + " Гм";
            }
            if (meters >= 1000000.0)
            {
                return FormatNumber(meters / 1000000.0, 2) + " Мм";
            }
            if (meters >= 1000.0)
            {
                return FormatNumber(meters / 1000.0, 1) + " км";
            }
            return FormatNumber(meters, 0) + " м";
        }

        private void RefreshBodies(bool resetSelection)
        {
            if (FlightGlobals.Bodies == null)
            {
                return;
            }

            CelestialBody previousOrigin = null;
            CelestialBody previousDestination = null;
            List<CelestialBody> previousTargetBodies = new List<CelestialBody>();
            List<TargetVisitMode> previousTargetModes = new List<TargetVisitMode>();
            if (bodies.Count > 0)
            {
                previousOrigin = bodies[WrapIndex(originIndex, bodies.Count)];
                previousDestination = bodies[WrapIndex(destinationIndex, bodies.Count)];
                for (int i = 0; i < missionTargets.Count; i++)
                {
                    previousTargetBodies.Add(bodies[WrapIndex(missionTargets[i].BodyIndex, bodies.Count)]);
                    previousTargetModes.Add(missionTargets[i].Mode);
                }
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
                originIndex = -1;
                if (FlightGlobals.ActiveVessel != null && FlightGlobals.ActiveVessel.mainBody != null)
                {
                    originIndex = bodies.IndexOf(FlightGlobals.ActiveVessel.mainBody);
                }
                if (originIndex < 0) originIndex = 0;

                destinationIndex = FindDefaultDestinationIndex(originIndex);
            }
            else
            {
                if (previousOrigin != null && bodies.Contains(previousOrigin)) originIndex = bodies.IndexOf(previousOrigin);
                else originIndex = WrapIndex(originIndex, bodies.Count);

                if (previousDestination != null && bodies.Contains(previousDestination)) destinationIndex = bodies.IndexOf(previousDestination);
                else destinationIndex = WrapIndex(destinationIndex, bodies.Count);

                if (destinationIndex == originIndex)
                {
                    destinationIndex = FindDefaultDestinationIndex(originIndex);
                }
            }

            if (previousTargetBodies.Count > 0)
            {
                missionTargets.Clear();
                for (int i = 0; i < previousTargetBodies.Count; i++)
                {
                    int restoredIndex = bodies.IndexOf(previousTargetBodies[i]);
                    if (restoredIndex < 0)
                    {
                        restoredIndex = WrapIndex(destinationIndex, bodies.Count);
                    }

                    missionTargets.Add(new TargetSelection
                    {
                        BodyIndex = restoredIndex,
                        Mode = previousTargetModes[i]
                    });
                }
                selectedTargetIndex = Mathf.Clamp(selectedTargetIndex, 0, missionTargets.Count - 1);
                SyncDestinationFromTargets();
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

        private int FindDefaultDestinationIndex(int startIndex)
        {
            if (bodies.Count <= 1)
            {
                return WrapIndex(startIndex, bodies.Count);
            }

            CelestialBody origin = bodies[WrapIndex(startIndex, bodies.Count)];
            for (int offset = 1; offset < bodies.Count; offset++)
            {
                int candidateIndex = WrapIndex(startIndex + offset, bodies.Count);
                CelestialBody candidate = bodies[candidateIndex];
                LegGeometry geometry;
                if (candidate != origin && TryGetLegGeometry(origin, candidate, out geometry))
                {
                    return candidateIndex;
                }
            }

            return WrapIndex(startIndex + 1, bodies.Count);
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
                departSamples = 34;
                tofSamples = 28;
            }
            else if (qualityIndex == 1)
            {
                departSamples = 52;
                tofSamples = 40;
            }
            else
            {
                departSamples = 84;
                tofSamples = 64;
            }
        }

        private void GetGravityAssistQualitySamples(out int departSamples, out int legSamples, out int beamWidth)
        {
            if (qualityIndex <= 0)
            {
                departSamples = 6;
                legSamples = 4;
                beamWidth = 8;
            }
            else if (qualityIndex == 1)
            {
                departSamples = 10;
                legSamples = 6;
                beamWidth = 12;
            }
            else
            {
                departSamples = 16;
                legSamples = 9;
                beamWidth = 20;
            }
        }

        private int AdjustGravityAssistDepartSamples(int baseSamples, double searchSpan)
        {
            double years = YearSeconds() > 0.0 ? searchSpan / YearSeconds() : 0.0;
            if (!IsFinite(years) || years <= 8.0)
            {
                return baseSamples;
            }

            double targetStepYears = qualityIndex <= 0 ? 1.6 : (qualityIndex == 1 ? 1.1 : 0.75);
            int densitySamples = (int)Math.Ceiling(years / targetStepYears) + 1;
            int cap = qualityIndex <= 0 ? 220 : (qualityIndex == 1 ? 360 : 640);
            return Math.Max(baseSamples, Math.Min(cap, densitySamples));
        }

        private static void GetAutomaticTofRange(CelestialBody origin, CelestialBody destination, out double minTof, out double maxTof)
        {
            double day = DaySeconds();
            double hohmann = EstimateLegTime(origin, destination);

            LegGeometry geometry;
            if (TryGetLegGeometry(origin, destination, out geometry) &&
                (geometry.Kind == LegKind.ParentToChild || geometry.Kind == LegKind.ChildToParent))
            {
                minTof = Math.Max(day * 0.25, hohmann * 0.35);
                maxTof = Math.Max(minTof + day, hohmann * 2.60);
                return;
            }

            minTof = Math.Max(day * 5.0, hohmann * 0.38);
            maxTof = Math.Max(minTof + day * 10.0, hohmann * 1.85);
        }

        private static double CircularOrbitToHyperbolaDV(CelestialBody body, double altitude, double vinf)
        {
            return body != null
                ? AstrodynamicsMath.CircularOrbitToHyperbolaDV(body.Radius, body.gravParameter, altitude, vinf)
                : double.PositiveInfinity;
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

        private sealed class CandidateBody
        {
            public CelestialBody Body;
            public double Score;
            public double SemiMajorAxis;
        }

        private enum LegKind
        {
            SameParent,
            CommonAncestor,
            ParentToChild,
            ChildToParent
        }

        private sealed class LegGeometry
        {
            public CelestialBody From;
            public CelestialBody To;
            public CelestialBody CentralBody;
            public LegKind Kind;
        }

        private enum TargetVisitMode
        {
            Flyby,
            Capture
        }

        private struct TargetSelection
        {
            public int BodyIndex;
            public TargetVisitMode Mode;
        }

        private sealed class MissionTarget
        {
            public CelestialBody Body;
            public TargetVisitMode Mode;
            public int OriginalIndex;
        }

        private struct RouteNode
        {
            public CelestialBody Body;
            public TargetVisitMode Mode;
            public bool IsMandatory;
            public int TargetIndex;
        }

        private sealed class RouteTemplate
        {
            public RouteNode[] Nodes;
            public MissionTarget[] OrderedTargets;
        }

        private sealed class RouteSearchSeed
        {
            public TransferResult Result;
            public RouteTemplate Template;
            public double DepartUT;
            public int PorkchopJ;
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
            public bool UseActiveVesselStart;
            public Orbit ActiveVesselOrbit;
            public string ActiveVesselName;
        }

        private sealed class GravityRouteSearchRequest
        {
            public CelestialBody Origin;
            public CelestialBody Destination;
            public List<MissionTarget> Targets;
            public bool AutoTargetOrder;
            public double ParkingAltitude;
            public double CaptureAltitude;
            public double StartUT;
            public double SearchSpan;
            public double MaxMissionTime;
            public int MaxFlybys;
            public int DepartSamples;
            public int LegSamples;
            public int BeamWidth;
            public bool IncludeCapture;
            public bool UseMoonAssists;
            public bool UseActiveVesselStart;
            public Orbit ActiveVesselOrbit;
            public string ActiveVesselName;
        }

        private sealed class FlybyEvaluation
        {
            public bool IsValid;
            public double PoweredDV;
            public double RequiredTurnDeg;
            public double FreeTurnDeg;
            public double PeriapsisRadius;
            public string AssistName;
            public double AssistUT;
        }

        private sealed class CaptureEvaluation
        {
            public bool IsValid;
            public double CaptureDV;
            public double PoweredDV;
            public double TotalDV;
            public double PostAssistVInf;
            public double RequiredTurnDeg;
            public double FreeTurnDeg;
            public string AssistName;
            public double AssistUT;
        }

        private sealed class ManeuverCandidate
        {
            public bool IsValid;
            public bool EncounterFound;
            public double UT;
            public double ArrivalUT;
            public double EncounterUT;
            public double MissDistance;
            public double TargetSoi;
            public bool RequiresEncounter;
            public double Score;
            public double IncomingVInfError;
            public double OutgoingVInfError;
            public double FlybyPeriapsisRadius;
            public double NextTargetMissDistance;
            public double NextTargetSoi;
            public Vector3d DeltaV;

            public double DeltaVMagnitude
            {
                get { return DeltaV.magnitude; }
            }
        }

        private sealed class RouteLegResult
        {
            public CelestialBody From;
            public CelestialBody To;
            public CelestialBody CentralBody;
            public LegKind Kind;
            public double DepartUT;
            public double ArrivalUT;
            public double TimeOfFlight;
            public Vector3d StartPosition;
            public Vector3d StartVelocity;
            public Vector3d EndPosition;
            public Vector3d StartBodyVelocity;
            public Vector3d EndBodyVelocity;
            public bool HasTargetOffset;
            public Vector3d TargetOffset;
            public Vector3d DepartureVInfVector;
            public Vector3d ArrivalVInfVector;
            public double DepartureVInf;
            public double ArrivalVInf;
            public bool HasDepartureDVOverride;
            public double DepartureDVOverride;
            public bool HasInsertionDVOverride;
            public double InsertionDVOverride;
            public bool StartsFromActiveVessel;
            public Orbit ActiveVesselOrbit;
            public Vector3d ActiveVesselWorldDeltaV;
            public double ActiveVesselDepartureDV;
            public double FlybyPoweredDV;
            public double RequiredTurnDeg;
            public double FreeTurnDeg;
            public double PlannedFlybyPeriapsisRadius;
            public double PlannedCapturePeriapsisRadius;
            public string FlybyAssistName;
            public double CaptureDV;
            public double CapturePoweredDV;
            public double CapturePostAssistVInf;
            public double CaptureTurnDeg;
            public double CaptureFreeTurnDeg;
            public string CaptureAssistName;
            public bool LongWay;

            public RouteLegResult Clone()
            {
                return new RouteLegResult
                {
                    From = From,
                    To = To,
                    CentralBody = CentralBody,
                    Kind = Kind,
                    DepartUT = DepartUT,
                    ArrivalUT = ArrivalUT,
                    TimeOfFlight = TimeOfFlight,
                    StartPosition = StartPosition,
                    StartVelocity = StartVelocity,
                    EndPosition = EndPosition,
                    StartBodyVelocity = StartBodyVelocity,
                    EndBodyVelocity = EndBodyVelocity,
                    HasTargetOffset = HasTargetOffset,
                    TargetOffset = TargetOffset,
                    DepartureVInfVector = DepartureVInfVector,
                    ArrivalVInfVector = ArrivalVInfVector,
                    DepartureVInf = DepartureVInf,
                    ArrivalVInf = ArrivalVInf,
                    HasDepartureDVOverride = HasDepartureDVOverride,
                    DepartureDVOverride = DepartureDVOverride,
                    HasInsertionDVOverride = HasInsertionDVOverride,
                    InsertionDVOverride = InsertionDVOverride,
                    StartsFromActiveVessel = StartsFromActiveVessel,
                    ActiveVesselOrbit = ActiveVesselOrbit,
                    ActiveVesselWorldDeltaV = ActiveVesselWorldDeltaV,
                    ActiveVesselDepartureDV = ActiveVesselDepartureDV,
                    FlybyPoweredDV = FlybyPoweredDV,
                    RequiredTurnDeg = RequiredTurnDeg,
                    FreeTurnDeg = FreeTurnDeg,
                    PlannedFlybyPeriapsisRadius = PlannedFlybyPeriapsisRadius,
                    PlannedCapturePeriapsisRadius = PlannedCapturePeriapsisRadius,
                    FlybyAssistName = FlybyAssistName,
                    CaptureDV = CaptureDV,
                    CapturePoweredDV = CapturePoweredDV,
                    CapturePostAssistVInf = CapturePostAssistVInf,
                    CaptureTurnDeg = CaptureTurnDeg,
                    CaptureFreeTurnDeg = CaptureFreeTurnDeg,
                    CaptureAssistName = CaptureAssistName,
                    LongWay = LongWay
                };
            }
        }

        private sealed class RouteState
        {
            public double Time;
            public double Cost;
            public double EjectionDV;
            public double InsertionDV;
            public double FlybyDV;
            public Vector3d IncomingVInf;
            public List<double> Times;
            public List<RouteLegResult> Legs;

            public RouteState Clone()
            {
                RouteState clone = new RouteState
                {
                    Time = Time,
                    Cost = Cost,
                    EjectionDV = EjectionDV,
                    InsertionDV = InsertionDV,
                    FlybyDV = FlybyDV,
                    IncomingVInf = IncomingVInf,
                    Times = new List<double>(Times),
                    Legs = new List<RouteLegResult>()
                };

                for (int i = 0; i < Legs.Count; i++)
                {
                    clone.Legs.Add(Legs[i].Clone());
                }

                return clone;
            }
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
            public double FlybyDV;
            public bool GravityRoute;
            public bool LongWay;
            public CelestialBody[] RouteBodies;
            public RouteNode[] RouteNodes;
            public double[] EncounterUTs;
            public RouteLegResult[] RouteLegs;
            public string RouteName;
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
