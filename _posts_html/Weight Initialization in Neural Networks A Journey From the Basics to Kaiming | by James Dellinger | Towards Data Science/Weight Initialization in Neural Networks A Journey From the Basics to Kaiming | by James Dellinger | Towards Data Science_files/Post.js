(self.webpackChunklite=self.webpackChunklite||[]).push([[2332],{53764:(n,e,t)=>{"use strict";t.r(e),t.d(e,{PostHandlerQuery:()=>d,PostHandler:()=>v});var r=t(28655),o=t.n(r),i=t(71439),s=t(67294),c=t(12291),u=t(27572),a=t(12450),l=t(68831),p=t(19307);function f(){var n=o()(["\n  query PostHandler(\n    $postId: ID!\n    $postMeteringOptions: PostMeteringOptions\n    $includePostInternalLinks: Boolean!\n    $includePostRecirc: Boolean = false\n    $includeSequenceRecirc: Boolean = false\n    $postRecircPaging: PaginationLimit\n  ) {\n    meterPost(postId: $postId, postMeteringOptions: $postMeteringOptions) {\n      __typename\n      ... on MeteringInfo {\n        ...PostScreen_meteringInfo\n      }\n    }\n    postResult(id: $postId) {\n      __typename\n      ...SharedPostHandler_postResult\n      ... on Post {\n        ...PostScreen_post\n      }\n    }\n  }\n  ","\n  ","\n  ","\n"]);return f=function(){return n},n}var d=(0,i.Ps)(f(),l.De,a.J,l.m6),v=(0,c.$j)((function(n){return{recircOptions:n.config.recircOptions}}))((function(n){var e,t,r=n.match,o=n.recircOptions,i=(0,p.o)(),c=(i?null==o||null===(e=o.v2)||void 0===e?void 0:e.limit:null==o||null===(t=o.v1)||void 0===t?void 0:t.limit)||3;return s.createElement(a.N,{match:r,query:d,extraVariables:{includePostInternalLinks:i,postRecircPaging:c}},(function(n){var e=n.meterPost,t=n.postId,r=n.postResult,o=n.viewer;return s.createElement(u.cW,{source:{name:"post_page",postId:t}},s.createElement(l.gc,{viewer:o,meteringInfo:e,post:r}))}))}))},63012:(n,e,t)=>{var r=t(97786),o=t(10611),i=t(71811);n.exports=function(n,e,t){for(var s=-1,c=e.length,u={};++s<c;){var a=e[s],l=r(n,a);t(l,a)&&o(u,i(a,n),l)}return u}},45220:n=>{n.exports=function(n){return null===n}},94885:n=>{n.exports=function(n){if("function"!=typeof n)throw new TypeError("Expected a function");return function(){var e=arguments;switch(e.length){case 0:return!n.call(this);case 1:return!n.call(this,e[0]);case 2:return!n.call(this,e[0],e[1]);case 3:return!n.call(this,e[0],e[1],e[2])}return!n.apply(this,e)}}},14176:(n,e,t)=>{var r=t(67206),o=t(94885),i=t(35937);n.exports=function(n,e){return i(n,o(r(e)))}},35937:(n,e,t)=>{var r=t(29932),o=t(67206),i=t(63012),s=t(46904);n.exports=function(n,e){if(null==n)return{};var t=r(s(n),(function(n){return[n]}));return e=o(e),i(n,t,(function(n,t){return e(n,t[0])}))}}}]);
//# sourceMappingURL=https://stats.medium.build/lite/sourcemaps/Post.b2e9c91e.chunk.js.map