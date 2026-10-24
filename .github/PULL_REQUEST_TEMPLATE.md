## 📝 Summary

<!-- What does this PR do, and why? Keep it to 2–3 sentences. -->

Closes #<!-- issue number, or delete this line -->

## 🔖 Type of change

- [ ] 🐛 Bug fix
- [ ] ✨ New signal / value
- [ ] 🎨 Dashboard / UI change
- [ ] ⚡ Performance or stability
- [ ] 📚 Documentation
- [ ] 🧹 Refactor / cleanup
- [ ] 💥 Breaking change

## 🔍 Details

<!-- What changed? Mention affected dashboard categories, JSON endpoints or sections of the sketch. -->

## 🛵 Testing

**Hardware**

| | |
|---|---|
| Board | <!-- e.g. ESP32-C3 SuperMini --> |
| CAN transceiver | <!-- e.g. SN65HVD230 --> |
| Arduino ESP32 core | <!-- e.g. 3.x.x --> |

**Vehicle**

| | |
|---|---|
| Model / variant | <!-- e.g. 450X Gen 3 / Rizta --> |
| Vehicle firmware (if known) | |
| Tested while | <!-- parked / charging / riding --> |

**Checklist**

- [ ] Sketch compiles with no new warnings
- [ ] Flashed and tested on real hardware (or a CAN log replay; state which)
- [ ] Dashboard reaches **ONLINE** and the affected category displays correctly
- [ ] Serial `[DIAG]` shows `rx_missed` / `rx_overrun` at 0 during the test
- [ ] No firmware path queues or transmits frames on the bus

## 📡 New or changed signals *(if applicable)*

| Dashboard label | Unit | Source of confirmation | Confidence |
|---|---|---|---|
| | | <!-- own test / existing project / log --> | <!-- confirmed / estimated / hypothesis --> |

- [ ] Marked `Estimated` or `Hypothesis` on the dashboard if not fully confirmed
- [ ] Source credited if signal data came from another project

## 📸 Screenshots *(UI changes)*

<!-- Before / after -->

## 📚 Documentation

- [ ] `docs/VALUES_REFERENCE.md` updated
- [ ] `docs/ALGORITHMS.md` updated (if decoding logic changed)
- [ ] `README.md` updated (if user-facing)

## ✅ Final checklist

- [ ] My code follows the existing style of the sketch
- [ ] I checked payload-length limits for every new frame handler
- [ ] I did not commit personal data (VIN, Wi-Fi passwords, logs with identifiers)
- [ ] I have the right to contribute this code under the project licence
