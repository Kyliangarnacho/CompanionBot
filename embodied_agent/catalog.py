"""Data-owned names/relations; small-catalog retrieval behind an async provider.

No pose, map coordinates or inventory inference. SQLite/API/MCP adapters implement
CatalogProvider, while the Agent consumes the same typed search/resolve result.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import re
from pathlib import Path
import time
from typing import Annotated, Callable, Literal, Protocol
import unicodedata
from uuid import uuid4

from pydantic import BaseModel, BeforeValidator, ConfigDict, Field, model_validator
from rapidfuzz.fuzz import ratio


def normalized(value: str) -> str:
    return "".join(c.casefold() for c in unicodedata.normalize("NFKC", value) if c.isalnum())


def json_object(value):
    """Qwen may encode nested tool objects as JSON strings. Validate after decoding."""
    if isinstance(value, str):
        if len(value) > 4096:
            raise ValueError("tool_object_too_large")
        value = json.loads(value)
    if not isinstance(value, dict):
        raise ValueError("tool_object_required")
    return value


StringMap = Annotated[dict[str, str], BeforeValidator(json_object)]
QueryTerm = Annotated[str, Field(min_length=1, max_length=160)]
QueryVariants = Annotated[tuple[QueryTerm, ...], Field(max_length=3)]


class NamedRecord(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)
    id: str = Field(min_length=1, max_length=80)
    name: str = Field(min_length=1)
    aliases: tuple[str, ...] = ()


class Destination(NamedRecord):
    active: bool = True


class Category(NamedRecord):
    parent_id: str | None = None
    destination_id: str | None = None


class CatalogEntity(NamedRecord):
    sku: str | None = None
    brand_id: str | None = None
    category_id: str | None = None
    attributes: dict[str, str] = Field(default_factory=dict)
    location_label: str | None = None
    destination_id: str | None = None
    description: str = ""


class AttributeDefinition(NamedRecord):
    source_field: Literal["attributes", "brand_id"] = "attributes"
    values: tuple[NamedRecord, ...] = ()


class CatalogDocument(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)
    version: Literal[2] = 2
    notice: str
    destinations: tuple[Destination, ...]
    brands: tuple[NamedRecord, ...] = ()
    categories: tuple[Category, ...] = ()
    items: tuple[CatalogEntity, ...] = ()
    attribute_definitions: tuple[AttributeDefinition, ...] = ()

    @model_validator(mode="after")
    def references(self):
        groups = (self.destinations, self.brands, self.categories, self.items)
        for group in groups:
            if len({r.id for r in group}) != len(group):
                raise ValueError("duplicate_catalog_id")
        skus = [r.sku for r in self.items if r.sku]
        if len(set(skus)) != len(skus):
            raise ValueError("duplicate_sku")
        destinations = {r.id for r in self.destinations}
        brands = {r.id for r in self.brands}
        categories = {r.id: r for r in self.categories}
        for record in (*self.items, *self.categories):
            if record.destination_id and record.destination_id not in destinations:
                raise ValueError("unknown_destination_reference")
        for item in self.items:
            if item.brand_id and item.brand_id not in brands:
                raise ValueError("unknown_brand_reference")
            if item.category_id and item.category_id not in categories:
                raise ValueError("unknown_category_reference")
        for category in self.categories:
            seen, current = set(), category
            while current.parent_id:
                if current.id in seen or current.parent_id not in categories:
                    raise ValueError("invalid_category_hierarchy")
                seen.add(current.id)
                current = categories[current.parent_id]
        return self


class CatalogQuery(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)
    query: str = Field(min_length=1, max_length=160)
    query_variants: QueryVariants = ()
    target_kind: Literal["auto", "product", "category", "destination", "area"] = "auto"
    brand: str | None = None
    category: str | None = None
    sku: str | None = None
    attributes: dict[str, str] = Field(default_factory=dict)

    @model_validator(mode="after")
    def bounded_terms(self):
        if any(not normalized(term) for term in (self.query, *self.query_variants)):
            raise ValueError("empty_search_term")
        return self


class MatchSource(BaseModel):
    """Retrieval evidence, never a user-intent confidence or authorization."""
    model_config = ConfigDict(extra="forbid", frozen=True)
    query_index: int = Field(ge=0, le=3)
    field: str
    method: Literal["exact", "substring", "tokens", "fuzzy", "sku_filter"]
    score: float = Field(ge=0, le=1)


class CatalogMatch(MatchSource):
    record_kind: Literal["category", "destination"]
    record_id: str


class Candidate(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)
    entity: CatalogEntity
    score: float = Field(ge=0, le=1)
    brand: NamedRecord | None = None
    category_path: tuple[Category, ...] = ()
    matches: tuple[MatchSource, ...] = ()


class CatalogSearch(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)
    revision: str
    notice: str
    items: tuple[Candidate, ...] = ()
    categories: tuple[Category, ...] = ()
    destinations: tuple[Destination, ...] = ()
    total_count: int = 0
    truncated: bool = False
    queries: tuple[str, ...] = ()
    matches: tuple[CatalogMatch, ...] = ()  # Category/destination evidence.
    total_category_count: int = 0
    total_destination_count: int = 0
    ignored_query_variants_reason: str | None = None
    region_entity_conflict: bool | None = None  # None: provider has not checked the complete union.


class CatalogProvider(Protocol):
    """search accepts a bounded query batch, returns bounded fused candidates.

    No scan/index/ranking implementation is required by this interface. External
    providers can use indexed SQL, lexical/embedding hybrid retrieval or APIs.
    """
    @property
    def revision(self) -> str: ...
    async def search(self, query: CatalogQuery) -> CatalogSearch: ...
    def destination_valid(self, destination_id: str) -> bool: ...
    def entity_valid(self, entity_id: str) -> bool: ...
    def category_valid(self, category_id: str) -> bool: ...
    def destination_ids(self) -> frozenset[str]: ...
    def query_fields(self) -> tuple[AttributeDefinition, ...]: ...


class JsonCatalogProvider:
    """Validated small snapshot. Revision also detects nested attribute mutation."""
    def __init__(self, document: CatalogDocument):
        self.document = CatalogDocument.model_validate(document.model_dump())
        self.brands = {r.id: r for r in document.brands}
        self.categories = {r.id: r for r in document.categories}
        self.destinations = {r.id: r for r in document.destinations}

    @property
    def revision(self):
        return hashlib.sha256(self.document.model_dump_json().encode()).hexdigest()[:16]

    @classmethod
    def load(cls, path: Path):
        return cls.from_dict(json.loads(path.read_text("utf-8")))

    @classmethod
    def from_dict(cls, data: dict):
        # v1 compatibility is confined to the adapter; never guess item locations.
        if data.get("version", 1) == 1:
            data = dict(data)
            data["destinations"] = [{"id": key, "name": name} for key, name in data["destinations"].items()]
            data["items"] = [dict(item, destination_id=item["id"] if any(
                d["id"] == item["id"] for d in data["destinations"]) else None) for item in data["items"]]
            data["version"] = 2
        return cls(CatalogDocument.model_validate(data))

    def destination_ids(self):
        return frozenset(key for key, r in self.destinations.items() if r.active)

    def query_fields(self):
        return self.document.attribute_definitions

    def attribute_matches(self, item, key, value):
        definition = next((d for d in self.document.attribute_definitions if self.matches(key, d)), None)
        if definition is None:
            return normalized(item.attributes.get(key, "")) == normalized(value)
        if definition.source_field == "brand_id":
            return item.brand_id is not None and self.matches(value, self.brands[item.brand_id])
        canonical = next((v.id for v in definition.values if self.matches(value, v)), value)
        return normalized(item.attributes.get(definition.name, "")) == normalized(canonical)

    def destination_valid(self, destination_id):
        return destination_id in self.destination_ids()

    def entity_valid(self, entity_id):
        return any(r.id == entity_id for r in self.document.items)

    def category_valid(self, category_id):
        return category_id in self.categories

    @staticmethod
    def terms(record):
        return (record.id, record.name, *record.aliases)

    @classmethod
    def matches(cls, text, record):
        return normalized(text) in {normalized(t) for t in cls.terms(record)}

    def lineage(self, category_id):
        result = []
        while category_id:
            category = self.categories[category_id]
            result.append(category)
            category_id = category.parent_id
        return result

    @staticmethod
    def covered_by_fields(text, values):
        """Bounded literal word segmentation across this candidate's real fields.

        This is lexical evidence, not synonym/semantic expansion. Handles joined
        names and constraints without requiring hand-maintained phrase aliases.
        """
        vocabulary = {v for _, v in values if v}
        reachable = {0}
        for start in range(len(text)):
            if start in reachable:
                for term in vocabulary:
                    if text.startswith(term, start):
                        reachable.add(start + len(term))
        return len(text) in reachable

    async def search(self, query: CatalogQuery) -> CatalogSearch:
        identifier_query = bool(re.fullmatch(r"[A-Za-z0-9_-]+", query.query.strip()) and
                                any(c in query.query for c in "_-"))
        # Identifier constraints are exact references: hypotheses cannot replace
        # an absent SKU/ID, even if they happen to name a real neighboring item.
        variants = () if query.sku or identifier_query else query.query_variants
        queries, seen = [], set()
        for term in (query.query, *variants):
            # Keep model-provided word boundaries: an unsegmented original and
            # a spaced multi-field query may normalize alike but recall differently.
            identity = (normalized(term), tuple(normalized(t) for t in term.split()))
            if identity not in seen:
                queries.append(term)
                seen.add(identity)
        ignored = "exact_identifier_constraint" if query.query_variants and (query.sku or identifier_query) else None
        categories, destinations, matches = {}, {}, []
        if not query.sku and not identifier_query:
            for index, term in enumerate(queries):
                for kind, records, found_records in (
                        ("category", self.document.categories, categories),
                        ("destination", self.document.destinations, destinations)):
                    for record in records:
                        if kind == "destination" and not record.active:
                            continue
                        if kind == "category" and query.category and not any(
                                self.matches(query.category, c) for c in self.lineage(record.id)):
                            continue
                        if self.matches(term, record):
                            found_records[record.id] = record
                            matches.append(CatalogMatch(record_kind=kind, record_id=record.id,
                                query_index=index, field=kind, method="exact", score=1))
        found_by_query = [[] for _ in queries]
        for item in self.document.items:
            brand = self.brands.get(item.brand_id)
            lineage = self.lineage(item.category_id)
            if query.brand and (brand is None or not self.matches(query.brand, brand)):
                continue
            if query.category and not any(self.matches(query.category, c) for c in lineage):
                continue
            if query.sku and normalized(query.sku) != normalized(item.sku or ""):
                continue
            if any(not self.attribute_matches(item, k, v) for k, v in query.attributes.items()):
                continue
            terms = [("entity", t) for t in self.terms(item)] + [("sku", item.sku or "")]
            terms += [("attribute:" + k, v) for k, v in item.attributes.items()]
            terms += [("brand", t) for t in self.terms(brand)] if brand else []
            terms += [("category", t) for c in lineage for t in self.terms(c)]
            terms += [("destination", t) for t in self.terms(self.destinations[item.destination_id])] if item.destination_id else []
            values = [(field, normalized(t)) for field, t in terms if t]
            for index, term in enumerate(queries):
                q = normalized(term)
                tokens = [normalized(t) for t in term.split() if normalized(t)]
                exact = next((field for field, v in values if q == v), None)
                contained = next((field for field, v in values if q in v), None)
                if query.sku:
                    score, method, field = 1.0, "sku_filter", "sku"
                elif exact:
                    score, method, field = 1.0, "exact", exact
                elif identifier_query:
                    continue
                elif contained:
                    score, method, field = .96, "substring", contained
                elif len(tokens) > 1 and all(any(t in v for _, v in values) for t in tokens):
                    score, method, field = .96, "tokens", "multiple_fields"
                elif self.covered_by_fields(q, values):
                    score, method, field = .96, "tokens", "multiple_fields"
                else:
                    score, field = max(((ratio(q, v) / 100, field) for field, v in values), default=(0, "entity"))
                    method = "fuzzy"
                if score >= .78:
                    evidence = MatchSource(query_index=index, field=field, method=method, score=round(score, 4))
                    found_by_query[index].append(Candidate(entity=item, score=evidence.score, brand=brand,
                        category_path=tuple(lineage), matches=(evidence,)))
        fused = {}
        for found in found_by_query:
            best = max((c.score for c in found), default=0)
            for candidate in found:
                if candidate.score < best - .05:
                    continue
                prior = fused.get(candidate.entity.id)
                fused[candidate.entity.id] = candidate if prior is None else prior.model_copy(update={
                    "score": max(prior.score, candidate.score), "matches": prior.matches + candidate.matches})
        found = list(fused.values())
        found.sort(key=lambda c: (-c.score, c.entity.id))
        # No global score winner: conflicting expansion hypotheses remain visible.
        count = len(found)
        category_records, destination_records = tuple(categories.values())[:30], tuple(destinations.values())[:30]
        returned = {("category", r.id) for r in category_records} | {("destination", r.id) for r in destination_records}
        region_conflict = bool(categories or destinations) and any(not (
            any(c.id in {p.id for p in item.category_path} for c in categories.values()) or
            any(item.entity.destination_id == d.id for d in destinations.values())) for item in found)
        return CatalogSearch(revision=self.revision, notice=self.document.notice,
            items=tuple(found[:30]), categories=category_records, destinations=destination_records,
            total_count=count, truncated=count > 30 or len(categories) > 30 or len(destinations) > 30,
            queries=tuple(queries), matches=tuple(m for m in matches if (m.record_kind, m.record_id) in returned),
            total_category_count=len(categories), total_destination_count=len(destinations),
            ignored_query_variants_reason=ignored, region_entity_conflict=region_conflict)


@dataclass(frozen=True)
class Resolution:
    resolution_id: str
    destination_id: str
    revision: str
    turn_id: str
    input_id: str
    expires_at_s: float
    entity_ids: tuple[str, ...] = ()
    category_id: str | None = None
    basis: str = "unique_entity"


@dataclass
class Pending:
    query: CatalogQuery
    entity_ids: frozenset[str]
    revision: str
    expires_at_s: float
    created_turn_id: str
    category_ids: frozenset[str] = frozenset()
    destination_ids: frozenset[str] = frozenset()


class CatalogResolver:
    """One pending clarification and one turn-bound, expiring guide capability."""
    def __init__(self, provider: CatalogProvider, *, clock: Callable = time.perf_counter, ttl_s=120):
        self.provider, self.clock, self.ttl_s = provider, clock, ttl_s
        self.pending: Pending | None = None
        self.resolution: Resolution | None = None

    def clear(self):
        self.pending = self.resolution = None

    def begin_turn(self):
        self.resolution = None
        if self.pending and (self.pending.expires_at_s < self.clock() or self.pending.revision != self.provider.revision):
            self.pending = None
            return True
        return False

    def pending_context(self):
        return {"entity_ids": sorted(self.pending.entity_ids), "query": self.pending.query.model_dump(),
                "expires_at_s": self.pending.expires_at_s} if self.pending else None

    async def lookup(self, query: CatalogQuery, *, turn_id: str, input_id: str, refine_pending=False):
        pending = self.pending
        self.resolution = None
        if refine_pending:
            if not pending or pending.expires_at_s < self.clock() or pending.revision != self.provider.revision:
                self.clear()
                return {"status": "REJECT", "reason": "pending_expired_or_invalid", "items": []}
            # Conditions accumulate; the new query is supplied by the Agent from the
            # user's supplement. Intersection prevents drifting to an unrelated SKU.
            prior = pending.query.model_dump()
            patch = query.model_dump(exclude_none=True)
            if query.target_kind == "auto":
                patch["target_kind"] = prior["target_kind"]
            patch["attributes"] = prior["attributes"] | query.attributes
            query = CatalogQuery.model_validate(prior | patch)
        result = await self.provider.search(query)
        if result.revision != self.provider.revision:
            self.clear()
            return {"status": "REJECT", "reason": "catalog_changed_during_search", "items": []}
        items = tuple(c for c in result.items if not refine_pending or c.entity.id in pending.entity_ids)
        categories = tuple(c for c in result.categories if not refine_pending or c.id in pending.category_ids)
        destinations = tuple(d for d in result.destinations if not refine_pending or d.id in pending.destination_ids)
        self.pending = None
        response = result.model_dump(mode="json")
        # Optional defaults/nulls add no candidate evidence. Omit them at the
        # Agent boundary; typed consumers restore the same compatibility defaults.
        response["items"] = [c.model_dump(mode="json", exclude_defaults=True, exclude_none=True) for c in items]
        response["categories"] = [c.model_dump(mode="json") for c in categories]
        response["destinations"] = [d.model_dump(mode="json") for d in destinations]
        response.update(status="NO_MATCH", reason="no_match", resolution=None)
        destination, entity_ids, category_id, basis = None, (), None, None
        has_constraints = bool(query.brand or query.sku or query.attributes)
        original_category = any(m.record_kind == "category" and m.query_index == 0 for m in result.matches)
        original_entity = any(m.query_index == 0 and m.method == "exact" and m.field in ("entity", "sku")
                              for c in items for m in c.matches)
        category_intent = original_category or not original_entity or query.target_kind == "category"
        conflicting_regions = len(categories) > 1 or len(destinations) > 1 or bool(categories and destinations and
            {c.destination_id for c in categories} != {d.id for d in destinations})
        conflicting_regions |= result.region_entity_conflict is True
        region_incomplete = result.total_category_count > len(result.categories) or result.total_destination_count > len(result.destinations)
        region_incomplete |= result.truncated and result.region_entity_conflict is None
        # A unique category match must not hide an expansion that recalled an
        # unrelated entity. Scores across queries do not confirm user intent.
        if categories or destinations:
            conflicting_regions |= any(not (
                any(c.id in {p.id for p in item.category_path} for c in categories) or
                any(item.entity.destination_id == d.id for d in destinations)) for item in items)
        if not has_constraints and (conflicting_regions or region_incomplete and (categories or destinations)):
            self.pending = Pending(query, frozenset(c.entity.id for c in items), result.revision,
                self.clock() + self.ttl_s, turn_id, frozenset(c.id for c in categories), frozenset(d.id for d in destinations))
            response.update(status="CLARIFY", reason="truncated_candidates" if result.truncated else "query_hypothesis_ambiguity")
        elif query.target_kind != "destination" and category_intent and not has_constraints and len(categories) == 1:
            category = categories[0]
            destination, category_id, basis = category.destination_id, category.id, "category_zone"
        elif query.target_kind in ("auto", "destination") and not has_constraints and len(destinations) == 1:
            destination, basis = destinations[0].id, "named_destination"
        elif items and query.target_kind not in ("category", "destination"):
            entities = [c.entity for c in items]
            dests = {r.destination_id for r in entities}
            confident = all(c.score >= .92 for c in items) or (query.target_kind == "area" and
                        has_constraints and all(c.score >= .78 for c in items))
            if not result.truncated and confident and (len(items) == 1 or
                    query.target_kind == "area" and len(dests) == 1):
                destination, entity_ids = entities[0].destination_id, tuple(r.id for r in entities)
                basis = "unique_entity" if len(items) == 1 else "shared_destination_area"
            else:
                self.pending = Pending(query, frozenset(r.id for r in entities), result.revision,
                    self.clock() + self.ttl_s, turn_id, frozenset(c.id for c in categories), frozenset(d.id for d in destinations))
                response.update(status="CLARIFY", reason="truncated_candidates" if result.truncated else
                                "product_ambiguity" if len(items) > 1 else "confirm_fuzzy_match")
        if destination and self.provider.destination_valid(destination):
            self.resolution = Resolution(uuid4().hex, destination, result.revision, turn_id, input_id,
                                         self.clock() + self.ttl_s, entity_ids, category_id, basis)
            response.update(status="RESOLVED", reason=basis, resolution=vars(self.resolution))
        elif basis:
            response.update(status="UNAVAILABLE", reason="no_reachable_destination")
        return response

    def valid(self, resolution_id, destination_id, turn_id):
        r = self.resolution
        return bool(r and resolution_id == r.resolution_id and destination_id == r.destination_id
                    and r.turn_id == turn_id and self.clock() <= r.expires_at_s
                    and r.revision == self.provider.revision and self.pending is None
                    and self.provider.destination_valid(destination_id)
                    and all(self.provider.entity_valid(i) for i in r.entity_ids)
                    and (r.category_id is None or self.provider.category_valid(r.category_id)))
