# How This System Differs From Prior Work

Prior systems surveyed fall into four categories:

**Button-based confirmation** — require patient to press a button to confirm 
intake, unsuitable for users with cognitive decline or motor impairment.

**Cloud-dependent systems** — require stable internet connectivity with no 
local fallback, impractical in rural or low-infrastructure settings.

**Heavy hardware platforms** — Raspberry Pi based, drawing 500–650mA at 
$40–$55+, unsuitable for battery-powered home deployment.

**No intelligent interaction** — basic scheduling and buzzer alerts only, 
no voice interface, no behavioral adaptation.

This system is the first to combine:
- On-device conversational AI (KWS + SLU) on MCU-class hardware
- Physical multi-sensor adherence verification
- Offline-first operation with opportunistic cloud sync
- Sub-$15 BOM

For the full systematic literature review comparing 14 prior systems
across methodology, results, and limitations check: [docs/paper_preprint.md](docs/paper_preprint.md)
