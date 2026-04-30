# Auto Transfer Window Planner для Kerbal Space Program 1

Небольшой мод для KSP 1.x, который добавляет в игру окно поиска трансферных окон и строит простую porkchop-карту по дате старта, времени полёта и delta-v. Мод берёт список небесных тел из текущей игры, поэтому может работать не только со stock-системой, но и с RSS, KSRSS и другими planet pack.

## Быстрый запуск

1. Установи Kerbal Space Program 1.x.
2. Запусти из корня проекта:

```bat
build.bat
```

Если KSP стоит не в стандартной Steam-папке, передай путь к игре:

```bat
build.bat "D:\Games\Kerbal Space Program"
```

Скрипт:

- найдёт DLL-файлы KSP в `KSP_x64_Data\Managed`;
- соберёт `AutoTransferWindowPlanner.dll`;
- положит готовый мод в `GameData\AutoTransferWindowPlanner`;
- соберёт zip в `dist\AutoTransferWindowPlanner-<version>.zip`;
- скопирует мод в `Kerbal Space Program — test\GameData`, если такая test-установка найдена, иначе в выбранную `Kerbal Space Program\GameData`.

После этого запусти игру. Кнопка мода появится в стандартном toolbar KSP в Space Center, Tracking Station, Flight или Map View.

## Ручная установка

После успешной сборки можно вручную скопировать папку:

```text
GameData\AutoTransferWindowPlanner
```

в папку:

```text
Kerbal Space Program\GameData
```

## Требования для сборки

- Windows;
- Kerbal Space Program 1.8-1.12;
- MSBuild из Visual Studio Build Tools или `dotnet msbuild`;
- DLL из установленной игры:
  - `Assembly-CSharp.dll`;
  - `Assembly-CSharp-firstpass.dll`;
  - `UnityEngine.dll`;
  - `UnityEngine.CoreModule.dll`;
  - `UnityEngine.IMGUIModule.dll`.

Готовый DLL не хранится в репозитории, потому что он собирается против DLL конкретной установленной копии KSP.

## Что делает мод

- добавляет кнопку в стандартный toolbar KSP;
- позволяет выбрать стартовое тело и несколько обязательных целей;
- для каждой цели поддерживает режим `Пролёт` или `Захват`;
- умеет автоматически искать порядок посещения целей и маршрут через гравитационные манёвры других тел;
- умеет учитывать один лунный assist-манёвр на планетную встречу;
- может показывать рассчитанную пунктирную траекторию в Map View / Tracking Station;
- может создать и подогнать первый манёвр вылета на активном аппарате в Flight / Map View после расчёта маршрута, используя текущую орбиту аппарата, planet-centered ejection и игровой patched-conics solver KSP;
- ищет оптимальную дату старта в заданном диапазоне лет;
- автоматически подбирает диапазон времени полёта вокруг гомановского перелёта;
- считает суммарную delta-v, delta-v вылета, delta-v захвата, `v∞`, `C3`, фазовый угол и параметры вылета.

В режиме `Авто маршрут через flyby` мод перебирает цепочки вида `Kerbin -> Eve -> Kerbin -> Jool` или `Earth -> Venus -> Earth -> Mars`, решает Lambert-задачу для каждого участка и проверяет, может ли тело или его луна повернуть входящий `v∞` в исходящий `v∞` бесплатным пролётом. Если нужного угла поворота не хватает, в итог добавляется оценочный powered-flyby штраф delta-v.

Мод поддерживает все орбитальные `CelestialBody`, загруженные KSP: stock-планеты и луны, Earth/Moon/Mars в RSS/KSRSS и аналогичные тела из planet pack. Примеры маршрутов: `Kerbin -> Duna`, `Kerbin -> Mun`, `Mun -> Minmus`, `Earth -> Mars`, `Earth -> Moon`, `Moon -> Earth`, `Moon -> Mars`.

Ограничение остаётся patched-conics: расчёт строится как последовательность двухтельных Lambert-участков, а не как полноценная n-body симуляция. Для прямых перелётов планета ↔ луна мод использует отдельную локальную оценку от/к круговой парковочной орбите.
